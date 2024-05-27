from enum import Enum


class WebotsRosProjects(Enum):
    webots_ros2_suv = 'webots_ros2_suv'
    webots_ros2_tesla = 'webots_ros2_tesla'
    webots_ros2_control = 'webots_ros2_control'
    webots_ros2_driver = 'webots_ros2_driver'
    webots_ros2_epuck = 'webots_ros2_epuck'
    webots_ros2_importer = 'webots_ros2_importer'
    webots_ros2_mavic = 'webots_ros2_mavic'
    webots_ros2_msgs = 'webots_ros2_msgs'
    webots_ros2_tests = 'webots_ros2_tests'
    webots_ros2_tiago = 'webots_ros2_tiago'
    webots_ros2_turtlebot = 'webots_ros2_turtlebot'
    webots_ros2_universal_robot = 'webots_ros2_universal_robot'

    @classmethod
    def get_choices(cls):
        """Генерирует выбор для БД"""
        return tuple((member.name, member.value) for member in cls)
