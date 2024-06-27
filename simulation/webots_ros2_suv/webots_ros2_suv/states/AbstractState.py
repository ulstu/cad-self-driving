import math
import os
import pathlib
import yaml
from ament_index_python.packages import get_package_share_directory
from ackermann_msgs.msg import AckermannDrive

class AbstractState:
    def __init__(self, node=None) -> None:
        self.node = node
        package_dir = get_package_share_directory('webots_ros2_suv')
        config_path = os.path.join(package_dir,
                                    pathlib.Path(os.path.join(package_dir, 'config', 'global_coords.yaml')))
        self.config = None
        with open(config_path) as file:
            self.config = yaml.full_load(file)
            self.ackerman_correction = self.config['ackerman_angle_correction']
            self.ackerman_proportional_coef = self.config['ackerman_proportional_coef']
            self.turn_angle_num_path_points = self.config['turn_angle_num_path_points']
            self.default_speed = self.config['default_speed']

    def on_event(self, event, world_model=None):
        raise NotImplementedError

    def drive(self, world_model, angle_points_count=5, speed=None):
        if speed is None:
            speed = self.default_speed
        angle_points_count = self.turn_angle_num_path_points
        if world_model.path and len(world_model.path) < angle_points_count:
            angle_points_count = len(world_model.path) - 1
        if world_model.gps_path is not None and len(world_model.gps_path) > 2:
            command_message = AckermannDrive()
            command_message.speed = float(speed)
            command_message.steering_angle = world_model.gps_car_turn_angle
        elif not world_model.path or angle_points_count <= 2:
            command_message = AckermannDrive()
            command_message.speed = 0.0
            command_message.steering_angle = 0.0
        else:
            angles = []
            for i in range(1, angle_points_count):
                p1, p2 = world_model.path[0], world_model.path[i]
                div = (p2[1] - p1[1]) ** 2 + (p2[0] - p1[0]) ** 2
                angles.append(math.asin((p2[0] - p1[0]) / math.sqrt(div)) if (div > 0.001) else 0)
            angle = sum(angles) / len(angles)
            error = angle - self.ackerman_correction   # !!!!!!!!!!! зависит от матрицы гомографии!!!!!!!!
            # self.log(f'ANGLE: {angle}')

            command_message = AckermannDrive()
            command_message.speed = float(speed)
            command_message.steering_angle = error / math.pi * self.ackerman_proportional_coef

        world_model.command_message = command_message

        self.log(f'command message: {command_message}')

        world_model.params['speed'] = command_message.speed
        world_model.params['steering'] = command_message.steering_angle
        world_model.params['is_pause'] = world_model.is_pause

        # with open('/home/hiber/angle.csv','a') as fd:
        #     fd.write(f'{command_message.speed},{command_message.steering_angle},{datetime.now()}\n')
        # command_message.steering_angle = 0.0
        # self._logger.info(f'angle: {angle}; diff: {error * p_coef}')

    def log(self, message):
        self.node._logger.info(message)

    def logi(self, message):
        self.node._logger.info('\033[92m' + message + '\033[0m')
    
    def logw(self, message):
        self.node._logger.info('\033[93m' + message + '\033[0m')
    
    def loge(self, message):
        self.node._logger.info('\033[91m' + message + '\033[0m')
 