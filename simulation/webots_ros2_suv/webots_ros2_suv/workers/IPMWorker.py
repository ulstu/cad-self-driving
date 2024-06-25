from AbstractWorker import AbstractWorker
import cv2
import datetime
import traceback
from webots_ros2_suv.lib.map_builder import MapBuilder
import numpy as np
from fastseg.image import colorize, blend
from ament_index_python.packages import get_package_share_directory
from webots_ros2_suv.lib.timeit import timeit
from scipy.spatial.transform import Rotation as R
import os
import yaml


PACKAGE_NAME = 'webots_ros2_suv'

class IPMWorker(AbstractWorker):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__( *args, **kwargs)
        package_dir = get_package_share_directory(PACKAGE_NAME)

        self.lidardata_path = os.path.join(package_dir, "config/lidardata.yaml")
        # with open(self.lidardata_path, "r") as file:
        #     lidardata_config = yaml.safe_load(file)
    
        # self.zed_pos = np.array([lidardata_config["zed_x"], lidardata_config["zed_y"], lidardata_config["zed_z"]])
        self.__map_builder = MapBuilder(model_path=f'{package_dir}/resource/yolov8l_barrels.pt',
                                        ipm_config=f'{package_dir}/config/ipm_config.yaml')

    def __save_image_files(self, labels, composited, source):
        range_image = self.__cur_range_image.copy()
        cv2.imshow("range", range_image)
        cv2.imshow("composited image", composited)
        if cv2.waitKey(25) & 0xFF == ord('q'):
           return
        img_filename = datetime.now().strftime("%Y%m%d-%H%M%S")
        base_path = "/home/hiber/"
        np.save(f"{base_path}{img_filename}_seg.npy", labels)
        np.save(f"{base_path}{img_filename}_range.npy", range_image)
        np.save(f"{base_path}{img_filename}_composited.npy", composited)
        np.save(f"{base_path}{img_filename}_source.npy", source)


    def on_event(self, event, scene=None):
        return None
    
    #@timeit
    def __process_frame(self, world_model):
        try:
            image = world_model.rgb_image
            image_seg = world_model.seg_image
            #image_depth = world_model.range_image

            image = self.__map_builder.resize_img(image)
            image_seg = self.__map_builder.resize_img(image_seg.astype('float32'))
            #image_depth = self.__map_builder.resize_img(image_depth.astype('float32'))


            results = self.__map_builder.detect_objects(image)
            cboxes = results[0].boxes.data.cpu()
            # resized_boxes = np.reshape(np.array(cboxes)[:, :4], (-1, 2, 2)) / np.array(image.shape[:2])[::-1] * np.array(world_model.rgb_image.shape[:2])[::-1]
            resized_boxes = np.reshape(np.array(cboxes)[:, :4], (-1, 2, 2))
            world_model.yolo_detected_objects = zip(resized_boxes, results[0].boxes.cls)
            
            #depths = self.__map_builder.calc_box_distance(results[0].boxes.data, image_depth)

            image_seg = self.__map_builder.remove_detected_objects(image_seg, cboxes)
            world_model.ipm_image = self.__map_builder.generate_ipm(image_seg, is_mono=False, need_cut=False, log=self.log)
            world_model.ipm_image = self.__map_builder.crop_ipm(world_model.ipm_image, log=self.log)
            tbs, widths = self.__map_builder.transform_boxes(cboxes)

            world_model.pov_point = (image.shape[0], int(image.shape[1] / 2))
            world_model.pov_point = self.__map_builder.calc_bev_point(world_model.pov_point)
            world_model.pov_point = (world_model.pov_point[0], world_model.pov_point[1] - 35)
            
            world_model.ipm_image = self.__map_builder.put_objects(world_model.ipm_image, tbs, widths, results)

            world_model.ipm_colorized = np.asarray(colorize(world_model.ipm_image))
            world_model.img_front_objects = results[0].plot()
            world_model.map_builder = self.__map_builder
            
            world_model.img_front_objects_lines_signs_prj = np.copy(world_model.img_front_objects_lines_signs)

            with open(self.lidardata_path, "r") as file:
                lidardata_config = yaml.safe_load(file)
    
            self.zed_pos = np.array([lidardata_config["zed_x"], -lidardata_config["zed_y"], lidardata_config["zed_z"]])

            for box in world_model.lidar_bounding_boxes:
                xmin, xmax = box[0]
                ymin, ymax = box[1]
                zmin, zmax = box[2]

                p = []
                for z in [zmin, zmax]:
                    for (x, dir) in [[xmin, 1], [xmax, -1]]:
                        for y in [ymin, ymax][::dir]:
                            point = np.array([x, y, z]) - self.zed_pos       
                            p.append(point)
                # 7.16  -5.88   1.13
                # 5.07  -6.00   2.06
                # -2.09 -0.12   0.93
                print(box[0])
                
                to_next = False
                p_front = []
                for point in p:
                    image_shape = np.array(world_model.img_front_objects_lines_signs_prj.shape[:2])[::-1]
                    size = np.array([lidardata_config["projection_size"], lidardata_config["projection_size"]])
                    
                    if point[2] <= 0:
                        to_next = True
                        break
                    
                    fov_angle = 90
                    _z = 1 / np.tan(fov_angle / 2)
                    point_front = (point[:2] * _z) / point[2]

                    point_front = point_front * (size / 2) + image_shape / 2
                    #point_front[1] = image_shape[1] - point_front[1]

                    point_front[0] = min(max(0, point_front[0]), world_model.img_front_objects_lines_signs_prj.shape[1])
                    point_front[1] = min(max(0, point_front[1]), world_model.img_front_objects_lines_signs_prj.shape[0])

                    p_front.append(np.array(point_front, dtype=np.int32))
                
                if to_next:
                    continue
                
                # front_edge  = [p_front[1], p_front[0], p_front[2], p_front[3]]
                # right_edge  = [p_front[2], p_front[6], p_front[7], p_front[3]]
                # top_edge    = [p_front[6], p_front[5], p_front[2], p_front[1]]
                # left_edge   = [p_front[1], p_front[5], p_front[4], p_front[0]]
                # bottom_edge = [p_front[0], p_front[3], p_front[7], p_front[4]]
                # back_edge   = [p_front[7], p_front[5], p_front[4], p_front[6]]

                front_edge  = [p_front[0], p_front[1], p_front[2], p_front[3]]
                right_edge  = [p_front[3], p_front[2], p_front[6], p_front[7]]
                top_edge    = [p_front[1], p_front[5], p_front[6], p_front[2]]
                left_edge   = [p_front[0], p_front[1], p_front[5], p_front[4]]
                bottom_edge = [p_front[0], p_front[4], p_front[7], p_front[3]]
                back_edge   = [p_front[4], p_front[5], p_front[6], p_front[7]]

                cv2.drawContours(world_model.img_front_objects_lines_signs_prj, [np.array(back_edge).astype(int)], contourIdx=-1, color=(50, 50, 50), thickness=-1)
                cv2.drawContours(world_model.img_front_objects_lines_signs_prj, [np.array(bottom_edge).astype(int)], contourIdx=-1, color=(50, 50, 50), thickness=-1)
                cv2.drawContours(world_model.img_front_objects_lines_signs_prj, [np.array(left_edge).astype(int)], contourIdx=-1, color=(50, 50, 50), thickness=-1)
                
                cv2.drawContours(world_model.img_front_objects_lines_signs_prj, [np.array(top_edge).astype(int)], contourIdx=-1, color=(170, 170, 170), thickness=-1)
                cv2.drawContours(world_model.img_front_objects_lines_signs_prj, [np.array(right_edge).astype(int)], contourIdx=-1, color=(190, 190, 190), thickness=-1)
                cv2.drawContours(world_model.img_front_objects_lines_signs_prj, [np.array(front_edge).astype(int)], contourIdx=-1, color=(220, 220, 220), thickness=-1)
                

            #colorized, track_ids = self.__map_builder.track_objects(results, colorized, self.__pos)
            return world_model

        except  Exception as err:
            super().error(''.join(traceback.TracebackException.from_exception(err).format()))
    
    def on_data(self, world_model):
        return self.__process_frame(world_model)


