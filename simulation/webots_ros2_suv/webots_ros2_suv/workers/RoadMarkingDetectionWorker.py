from AbstractWorker import AbstractWorker
import torch
import pathlib
import os
import numpy as np
import traceback
from PIL import Image
from ament_index_python.packages import get_package_share_directory
from fastseg import MobileV3Large
from fastseg.image import colorize, blend
from webots_ros2_suv.lib.timeit import timeit
from webots_ros2_suv.lib.lane_line_model import LaneLineModel
from webots_ros2_suv.lib.lane_line_model_utils import get_label_names, draw_lines, draw_segmentation, LaneLine, LaneMask, default_palette
from webots_ros2_suv.lib.map_builder import MapBuilder
from webots_ros2_suv.lib.LinesAnalizator import LinesAnalizator
import cv2
import yaml
from ultralytics import YOLO

PACKAGE_NAME = "webots_ros2_suv"
local_model_path = "resource/RMm/model.pt"
# local_model_config_path = "resource/lane_line_model/config.yaml"

package_dir = get_package_share_directory(PACKAGE_NAME)
project_settings_config_path = os.path.join(package_dir, "config/project_settings.yaml")
with open(project_settings_config_path, "r") as file:
    project_settings_config = yaml.safe_load(file)

class RoadMarkingDetectionWorker(AbstractWorker):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)

        package_dir = get_package_share_directory(PACKAGE_NAME)
        model_path = os.path.join(package_dir, local_model_path)
        # config_path = os.path.join(package_dir, local_model_config_path)

        self.model = YOLO(model_path)
        self.labels = self.model.names
        self.lines_analizator = LinesAnalizator()
        
        

    def on_event(self, event, scene=None):
        return None


    def on_data(self, world_model):
        try:
            if world_model:
                if project_settings_config["use_road_marking_detection"] == True:
                    img = Image.fromarray(world_model.rgb_image)

                    world_model.img_front_objects_lines_signs_markings = np.copy(world_model.img_front_objects_lines_signs)

                    results = self.model.predict(np.array(img), verbose=True)
                    # for mask in results[0].masks:
                    #     for xy in mask.xy:
                    #         cv2.drawContours(world_model.img_front_objects_lines_signs, [np.expand_dims(xy, 1).astype(int)], contourIdx=-1, color=0, thickness=-1)

                    masks = None
                    if results[0].masks is not None:
                        masks = results[0].masks
                    
                    labels = [self.model.names[int(label)] for label in results[0].boxes.cls]

                    world_model.detected_road_markings = list(zip(masks, labels))

                    background_alpha = 0.7
                    if results[0].masks is not None:
                        for xy in results[0].masks.xy:

                            # print("_*_" * 100)
                            # print(world_model.img_front_objects_lines_signs_markings)
                            # print("_*_" * 100)

                            # image_mask = np.zeros_like(world_model.img_front_objects_lines_signs_markings).astype(np.uint8)

                            # cv2.drawContours(image_mask, [np.expand_dims(xy, 1).astype(int)], 
                            #                  contourIdx=-1, 
                            #                  color=(255, 255, 255), thickness=-1)
                            
                            # indices = np.any(image_mask != np.array([0, 0, 0], dtype=np.uint8), axis=-1)
                            # world_model.img_front_objects_lines_signs_markings[indices] = cv2.addWeighted(world_model.img_front_objects_lines_signs_markings, 
                            #                                                                               1 - background_alpha, image_mask, background_alpha, 0, image_mask)[indices]

                            cv2.drawContours(world_model.img_front_objects_lines_signs_markings, [np.expand_dims(xy, 1).astype(int)], 
                                             contourIdx=-1, 
                                             color=(255, 210, 74), thickness=-1)

        except  Exception as err:
            super().error(''.join(traceback.TracebackException.from_exception(err).format()))
        
        return world_model