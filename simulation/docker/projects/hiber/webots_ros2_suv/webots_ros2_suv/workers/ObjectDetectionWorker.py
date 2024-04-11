from AbstractWorker import AbstractWorker
import cv2
from webots_ros2_suv.lib.map_builder import MapBuilder
from ament_index_python.packages import get_package_share_directory

PACKAGE_NAME = 'webots_ros2_suv'

class ObjectDetectionWorker(AbstractWorker):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__( *args, **kwargs)
        package_dir = get_package_share_directory(PACKAGE_NAME)

        self.__map_builder = MapBuilder(model_path=f'{package_dir}/resource/yolov8l.pt',
                                        ipm_config=f'{package_dir}/config/ipm_config.yaml')

    def on_event(self, event, scene=None):
        print("Emergency State")
        return None

    def on_data(self, world_model):
        super().log("ObjectDetectionWorker data received")
        return world_model

