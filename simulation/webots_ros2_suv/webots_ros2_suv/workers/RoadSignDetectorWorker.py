from AbstractWorker import AbstractWorker
from webots_ros2_suv.lib.sign_traffic_detector import ImageAnalyzer
from ament_index_python.packages import get_package_share_directory
import os
import pathlib
import numpy as np
from PIL import Image

PACKAGE_NAME = 'webots_ros2_suv'
local_path_to_cnn_model = "weights/model-ep50-signs16/"
local_path_to_seg_model = "mobilev3large-lraspp-sign.pt"
local_path_to_icons = "signs_icon/"


class RoadSignDetectorWorker(AbstractWorker):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__( *args, **kwargs)

        package_dir = get_package_share_directory(PACKAGE_NAME)
        path_to_cnn_model = os.path.join(package_dir, pathlib.Path(os.path.join(package_dir, 'resource', local_path_to_cnn_model)))
        path_to_seg_model = os.path.join(package_dir, pathlib.Path(os.path.join(package_dir, 'resource', local_path_to_seg_model)))
        path_to_icons = os.path.join(package_dir, pathlib.Path(os.path.join(package_dir, 'resource', local_path_to_icons)))

        self.detector = ImageAnalyzer(path_to_cnn_model,
                        path_to_seg_model,
                        path_to_icons,
                        is_video=False,
                        is_red=False,
                        is_correct_size=True,
                        correct_width=1000)

    def on_event(self, event, scene=None):
        print("Emergency State")
        return None

    def on_data(self, world_model):
        img = np.array(Image.fromarray(world_model.rgb_image))
        img = world_model.map_builder.resize_img(img)
        image_to_draw = np.copy(world_model.img_front_objects_lines)
        self.detector.plot_predictions(img, image_to_draw)

        world_model.img_front_objects_lines_signs = image_to_draw
        
        #super().log("RoadSignDetectorWorker data received")
        return world_model
