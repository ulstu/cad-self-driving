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
from webots_ros2_suv.lib.lidar_yolo_box import LidarYoloBox
from typing import List


PACKAGE_NAME = 'webots_ros2_suv'

def intersection_area(rect1, rect2):
    left_x_inteval = min(max(rect2[0][0] - rect1[0][0], 0), rect1[1][0] - rect1[0][0])
    right_x_inverval = min(max(rect1[1][0] - rect2[1][0], 0), rect1[1][0] - rect1[0][0])
    x_intersection = (rect1[1][0] - rect1[0][0]) - left_x_inteval - right_x_inverval

    left_y_inteval = min(max(rect2[0][1] - rect1[0][1], 0), rect1[1][1] - rect1[0][1])
    right_y_inverval = min(max(rect1[1][1] - rect2[1][1], 0), rect1[1][1] - rect1[0][1])
    y_intersection = (rect1[1][1] - rect1[0][1]) - left_y_inteval - right_y_inverval

    rect1_size = rect1[1] - rect1[0]
    rect2_size = rect2[1] - rect2[0]
    min_area = abs(min(rect1_size[0] * rect1_size[1], rect2_size[0] * rect2_size[1]))

    return (x_intersection * y_intersection) / min_area if min_area != 0 else 0

class IPMWorker(AbstractWorker):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__( *args, **kwargs)
        package_dir = get_package_share_directory(PACKAGE_NAME)

        # self.lidardata_path = os.path.join(package_dir, "config/lidardata.yaml")
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

            labels = [self.__map_builder._model.names[int(label)] for label in results[0].boxes.cls]
            world_model.yolo_detected_objects = list(zip(resized_boxes, labels))
            
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


            image_to_draw = np.copy(world_model.img_front_objects)

            # cv2.rectangle(world_model.img_front_objects_lines_signs_markings_prj, (0, 0), (100, 100), (255, 0, 0), thickness=-1)

            # with open(self.lidardata_path, "r") as file:
            #     lidardata_config = yaml.safe_load(file)
    
            # self.zed_pos = np.array([lidardata_config["zed_x"], -lidardata_config["zed_y"], lidardata_config["zed_z"]])
            world_model.lidar_yolo_boxes = []

            # for box in world_model.lidar_bounding_boxes:
                # xmin, xmax = box[0]
                # ymin, ymax = box[1]
                # zmin, zmax = box[2]

                # p = []
                # for z in [zmin, zmax]:
                #     for (x, dir) in [[xmin, 1], [xmax, -1]]:
                #         for y in [ymin, ymax][::dir]:
                #             point = np.array([x, y, z]) - self.zed_pos       
                #             p.append(point)
                
                # front_p1 = [image_to_draw.shape[1], image_to_draw.shape[0]] # min
                # front_p2 = [0, 0] # max
                # to_next = False
                # p_front = []
                # for point in p:
                #     image_shape = np.array(image_to_draw.shape[:2])[::-1]
                #     size = np.array([lidardata_config["projection_size"], lidardata_config["projection_size"]])
                    
                #     if point[2] <= 0:
                #         to_next = True
                #         break
                    
                #     fov_angle = 90
                #     _z = 1 / np.tan(fov_angle / 2)
                #     point_front = (point[:2] * _z) / point[2]

                #     point_front = point_front * (size / 2) + image_shape / 2

                #     point_front[0] = min(max(0, point_front[0]), image_to_draw.shape[1])
                #     point_front[1] = min(max(0, point_front[1]), image_to_draw.shape[0])

                #     front_p1[0] = min(front_p1[0], point_front[0])
                #     front_p1[1] = min(front_p1[1], point_front[1])

                #     front_p2[0] = max(front_p2[0], point_front[0])
                #     front_p2[1] = max(front_p2[1], point_front[1])

                #     p_front.append(np.array(point_front, dtype=np.int32))
                
                # if to_next:
                #     continue
                
                # front_p1 = np.array(front_p1).astype(int)
                # front_p2 = np.array(front_p2).astype(int)
                # lidar_box = np.array([front_p1, front_p2])

                # labels = self.__map_builder.get_labels()
                
                # to_next = True
                # for (box, box_cls) in world_model.yolo_detected_objects:
                #     inter_area = intersection_area(lidar_box, box)

                #     lidar_box_size = lidar_box[1] - lidar_box[0]
                #     yolo_box_size = box[1] - box[0]

                #     lidar_box_area = abs(lidar_box_size[0] * lidar_box_size[1])
                #     yolo_box_area = abs(yolo_box_size[0] * yolo_box_size[1])
                #     if lidar_box_area == 0 or yolo_box_area == 0:
                #         area_part = lidar_box_area if lidar_box_area != 0 else yolo_box_area
                #     else:
                #         area_part = min(lidar_box_area / yolo_box_area, yolo_box_area / lidar_box_area)
                    
                #     if inter_area >= lidardata_config["area_to_accept"] and area_part >= lidardata_config["area_part"]:
                #         p = np.array(p)
                #         xmin, xmax = [np.min(p[:, 0]), np.max(p[:, 0])]
                #         ymin, ymax = [np.min(p[:, 1]), np.max(p[:, 1])]
                #         zmin, zmax = [np.min(p[:, 2]), np.max(p[:, 2])]
                #         edges = np.array([[xmin, xmax], [ymin, ymax], [zmin, zmax]])
                #         lidar_yolo_box = LidarYoloBox(labels[int(box_cls)], box, p, edges)
                #         world_model.lidar_yolo_boxes.append(lidar_yolo_box)
                        
                #         to_next = False
                #         break

                # if to_next:
                #     continue

                # if lidardata_config["visualize_projection"]:
                #     front_edge  = [p_front[0], p_front[1], p_front[2], p_front[3]]
                #     right_edge  = [p_front[3], p_front[2], p_front[6], p_front[7]]
                #     top_edge    = [p_front[1], p_front[5], p_front[6], p_front[2]]
                #     left_edge   = [p_front[0], p_front[1], p_front[5], p_front[4]]
                #     bottom_edge = [p_front[0], p_front[4], p_front[7], p_front[3]]
                #     back_edge   = [p_front[4], p_front[5], p_front[6], p_front[7]]

                #     cv2.drawContours(image_to_draw, [np.array(back_edge).astype(int)], contourIdx=-1, color=(50, 50, 50), thickness=-1)
                #     cv2.drawContours(image_to_draw, [np.array(bottom_edge).astype(int)], contourIdx=-1, color=(50, 50, 50), thickness=-1)
                #     cv2.drawContours(image_to_draw, [np.array(left_edge).astype(int)], contourIdx=-1, color=(50, 50, 50), thickness=-1)
                    
                #     cv2.drawContours(image_to_draw, [np.array(top_edge).astype(int)], contourIdx=-1, color=(170, 170, 170), thickness=-1)
                #     cv2.drawContours(image_to_draw, [np.array(right_edge).astype(int)], contourIdx=-1, color=(190, 190, 190), thickness=-1)
                #     cv2.drawContours(image_to_draw, [np.array(front_edge).astype(int)], contourIdx=-1, color=(220, 220, 220), thickness=-1)

                #     cv2.rectangle(image_to_draw, front_p1, front_p2, color=(255, 0, 0), thickness=2)
            scale = lidardata_config["visual_scale"]
            for box in world_model.obstacles:
                xmin, xmax, ymin, ymax = box[8], box[9], box[10], box[11]
                xmin, xmax = (xmin  + lidardata_config["zed_x"]) * scale + world_model.pov_point[0], (xmax + lidardata_config["zed_x"]) * scale + world_model.pov_point[0]
                ymin, ymax = world_model.pov_point[1] - (ymin  + lidardata_config["zed_y"]) * scale, world_model.pov_point[1] - (ymax + lidardata_config["zed_y"]) * scale 
                self.logi(f"{xmin} {ymin} {xmax} {ymax}")
                world_model.ipm_colorized = cv2.rectangle(world_model.ipm_colorized, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (255, 0, 255), -1)

            #colorized, track_ids = self.__map_builder.track_objects(results, colorized, self.__pos)
            world_model.img_front_objects_prj = image_to_draw
            
            # world_model.ipm_colorized


            
            return world_model

        except  Exception as err:
            super().error(''.join(traceback.TracebackException.from_exception(err).format()))
    
    def on_data(self, world_model):
        return self.__process_frame(world_model)
