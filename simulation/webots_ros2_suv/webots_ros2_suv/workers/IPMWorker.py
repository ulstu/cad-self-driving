from AbstractWorker import AbstractWorker
import cv2
import datetime
import traceback
from webots_ros2_suv.lib.map_builder import MapBuilder
import numpy as np
from fastseg.image import colorize, blend
from ament_index_python.packages import get_package_share_directory
from webots_ros2_suv.lib.timeit import timeit


PACKAGE_NAME = 'webots_ros2_suv'

class IPMWorker(AbstractWorker):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__( *args, **kwargs)
        package_dir = get_package_share_directory(PACKAGE_NAME)

        self.__map_builder = MapBuilder(model_path=f'{package_dir}/resource/yolov8l.pt',
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
            tbs, widths = self.__map_builder.transform_boxes(cboxes)
            #depths = self.__map_builder.calc_box_distance(results[0].boxes.data, image_depth)

            image_seg = self.__map_builder.remove_detected_objects(image_seg, cboxes)
            world_model.ipm_image = self.__map_builder.generate_ipm(image_seg, is_mono=False, need_cut=False)

            world_model.pov_point = (image.shape[0], int(image.shape[1] / 2))
            world_model.pov_point = self.__map_builder.calc_bev_point(world_model.pov_point)
            world_model.pov_point = (world_model.pov_point[0], world_model.pov_point[1] - 15)
            world_model.ipm_image = self.__map_builder.put_objects(world_model.ipm_image, tbs, widths, results)

            ipm_image_height = self.__map_builder.calc_bev_point((0, image_seg.shape[0]))[1]

            world_model.ipm_colorized = np.asarray(colorize(world_model.ipm_image))[:ipm_image_height, :]
            world_model.ipm_image = world_model.ipm_image[:ipm_image_height, :]
            world_model.img_front_objects = results[0].plot()
            world_model.map_builder = self.__map_builder
            
            #colorized, track_ids = self.__map_builder.track_objects(results, colorized, self.__pos)
            return world_model

        except  Exception as err:
            super().error(''.join(traceback.TracebackException.from_exception(err).format()))
    
    def on_data(self, world_model):
        return self.__process_frame(world_model)


