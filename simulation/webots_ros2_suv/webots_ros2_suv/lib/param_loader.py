import yaml 
from ament_index_python.packages import get_package_share_directory

class ParamLoader(object):
    def __init__(self) -> None:
        package_dir = get_package_share_directory("webots_ros2_suv")
        with open(f'{package_dir}/config/topic_names.yaml', 'r') as file:
            self.config = yaml.safe_load(file)

    def get_param(self, name):
        return self.config.get(name, None)