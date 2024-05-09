from AbstractWorker import AbstractWorker
# from webots_ros2_suv.lib.sign_traffic_detector import ImageAnalyzer
import numpy as np
from PIL import Image
import torch
import pathlib
import os
import traceback
from ament_index_python.packages import get_package_share_directory
from fastseg import MobileV3Large
from fastseg.image import colorize, blend
from webots_ros2_suv.lib.timeit import timeit
from webots_ros2_suv.lib.lane_line_model import LaneLineModel
from webots_ros2_suv.lib.lane_line_model_utils import get_label_names, draw_lines, draw_segmentation, LaneLine, LaneMask, default_palette
from webots_ros2_suv.lib.map_builder import MapBuilder
import cv2

class RoadSignDetectorWorker(AbstractWorker):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__( *args, **kwargs)

        # self.detector = ImageAnalyzer(path_to_cnn_model="perception/signs_detector/weights/model-ep50-signs16/",
        #                 path_to_seg_model="perception/signs_detector/mobilev3large-lraspp.pt",
        #                 path_to_icons="perception/signs_detector/signs_icon/",
        #                 is_video=False,
        #                 is_red=False,
        #                 is_correct_size=True,
        #                 correct_width=1000)


    def on_event(self, event, scene=None):
        print("Emergency State")
        return None


    def on_data(self, world_model):
        # try:
        print("TEST__"*10)

        #     # img = np.array(Image.fromarray(world_model.rgb_image))
        #     # world_model.img_from_objects_lines_signs = img

        #     #self.detector.plot_predictions(img, world_model.img_from_objects_lines_signs)
        # except  Exception as err:
        #     super().error(''.join(traceback.TracebackException.from_exception(err).format()))
        
        return world_model
    