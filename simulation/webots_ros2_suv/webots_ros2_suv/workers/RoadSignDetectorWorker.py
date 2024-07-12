from AbstractWorker import AbstractWorker
from webots_ros2_suv.lib.sign_traffic_detector import ImageAnalyzer
from ament_index_python.packages import get_package_share_directory
import os
import pathlib
import numpy as np
from PIL import Image
import yaml
from webots_ros2_suv.lib.map_builder import MapBuilder
from webots_ros2_suv.lib.config_loader import GlobalConfigLoader

PACKAGE_NAME = 'webots_ros2_suv'
local_path_to_cnn_model = "resource/weights/model-ep50-signs16/"
local_path_to_seg_model = "resource/mobilev3large-lraspp-sign.pt"
local_path_to_tld_model = "resource/TLDm/model.pt"
local_path_to_sign_model = "resource/TLDm/model.pt"
local_path_to_icons = "resource/signs_icon/"
package_dir = get_package_share_directory(PACKAGE_NAME)


class RoadSignDetectorWorker(AbstractWorker):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__( *args, **kwargs)

        path_to_cnn_model = os.path.join(package_dir,  local_path_to_cnn_model)
        path_to_seg_model = os.path.join(package_dir,  local_path_to_seg_model)
        path_to_tld_model = os.path.join(package_dir,  local_path_to_tld_model)
        path_to_sign_model = os.path.join(package_dir, local_path_to_sign_model)

        path_to_icons = os.path.join(package_dir, local_path_to_icons)
        project_settings_config = GlobalConfigLoader("project_settings").data
    
        self.detector = ImageAnalyzer(path_to_cnn_model,
                        path_to_seg_model,
                        path_to_tld_model,
                        path_to_sign_model,
                        path_to_icons,
                        is_video=False,
                        is_red=True,
                        is_correct_size=True,
                        correct_width=1000, 
                        use_gpu=project_settings_config['use_gpu'])


    def on_event(self, event, scene=None):
        print("Emergency State")
        return None

    def on_data(self, world_model):
        if world_model:
            world_model.icons = self.detector.icons

            img = np.array(Image.fromarray(world_model.rgb_image))
            img = world_model.map_builder.resize_img(img)
            image_to_draw = np.copy(world_model.img_front_objects_prj_lines)
            self.detector.plot_predictions(img, world_model.yolo_detected_objects, image_to_draw, update_traffic_light_state=True)

            world_model.detected_signs = self.detector.detected_signs
            world_model.filtered_detected_signs = self.detector.filtered_detected_signs


            # print(f"DETECTED SIGNS FROM WORLD MODEL: {world_model.detected_signs}")

            world_model.img_front_objects_prj_lines_signs = image_to_draw

            #world_model.traffic_light_state = "red" if self.detector.is_red else "green"
            world_model.traffic_light_state = self.detector.traffic_light_state

            if self.detector.sign < 0:
                world_model.found_sign = None
            else:
                world_model.found_sign = [self.detector.sign, # Знак
                                    self.detector.labels[self.detector.sign],  # Название знака
                                    self.detector.get_icon(self.detector.labels[self.detector.sign])] # Иконка знака
        
        #super().log("RoadSignDetectorWorker data received")
        return world_model
