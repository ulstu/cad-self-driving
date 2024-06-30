import os
from ament_index_python.packages import get_package_share_directory
import pathlib
import yaml
from webots_ros2_suv.lib.config_loader import ConfigLoader

class AbstractWorker:
    def __init__(self, node=None) -> None:
        self.node = node
        self.config = ConfigLoader("global_coords").data

    def log(self, message):
        self.node._logger.info(message)
    
    def logi(self, message):
        self.log('\033[92m' + message + '\033[0m')
    
    def logw(self, message):
        self.log('\033[93m' + message + '\033[0m')
    
    def loge(self, message):
        self.log('\033[91m' + message + '\033[0m')

    def error(self, message):
        self.node._logger.error(message)

    def on_event(self, event, scene=None):
        raise NotImplementedError

    def on_data(self, world_model):
        raise NotImplementedError
    