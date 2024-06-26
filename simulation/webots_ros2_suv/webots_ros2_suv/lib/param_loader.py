import yaml 
from ament_index_python.packages import get_package_share_directory
from webots_ros2_suv.lib.config_loader import ConfigLoader

class ParamLoader(object):
    def __init__(self) -> None:
        self.config = ConfigLoader("devices_topics").data

    def get_param(self, name):
        return self.config.get(name, None)