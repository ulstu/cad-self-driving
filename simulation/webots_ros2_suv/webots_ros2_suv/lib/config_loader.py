import yaml 
import os
from ament_index_python.packages import get_package_share_directory

class ConfigLoader:
    def __init__(self, name="main", type='yaml'):
        package = get_package_share_directory("webots_ros2_suv")

        if not os.path.exists(f'{package}/config/main.yaml'):
            print("Main config file file not found. Solution will been stoped")
            exit(0)
        with open(f'{package}/config/main.yaml', 'r') as file:
            config = yaml.safe_load(file)
            cur_directory = os.path.join(package, "config" ,config.get("current", "simulator"))
            if not os.path.exists(os.path.join(cur_directory, f"{name}.{type}")):
                print(f"Config file file {cur_directory} not found")
            else:
                with open(os.path.join(cur_directory, f"{name}.{type}"), 'r') as file:
                    self.data = yaml.safe_load(file)

class GlobalConfigLoader:
    def __init__(self, name="main", type='yaml'):
        package = get_package_share_directory("webots_ros2_suv")
        cur_directory = os.path.join(package, "config")
        if not os.path.exists(os.path.join(cur_directory, f"{name}.{type}")):
            print(f"Config file file {cur_directory} not found. Solution will been stoped")
        else:
            with open(os.path.join(cur_directory, f"{name}.{type}"), 'r') as file:
                self.data = yaml.safe_load(file)



