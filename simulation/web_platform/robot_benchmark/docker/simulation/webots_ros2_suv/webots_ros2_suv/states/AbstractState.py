import math
from ackermann_msgs.msg import AckermannDrive

class AbstractState:

    def __init__(self, node=None) -> None:
        self.node = node

    def on_event(self, event, scene=None):
        raise NotImplementedError

    def drive(self, world_model, angle_points_count=3, speed=0.0):
        #self.log(f'PATH: {world_model.path}')
        
        if world_model.path and len(world_model.path) < angle_points_count:
            angle_points_count = len(world_model.path) - 1
        if not world_model.path or angle_points_count <= 2:
            command_message = AckermannDrive()
            command_message.speed = 0.0
            command_message.steering_angle = 0.0
            self.log('PATH TOO SHORT TO DRIVE')
        else:
            angles = []
            for i in range(1, angle_points_count):
                p1, p2 = world_model.path[0], world_model.path[i]
                div = (p2[1] - p1[1]) ** 2 + (p2[0] - p1[0]) ** 2
                angles.append(math.asin((p2[0] - p1[0]) / math.sqrt(div)) if (div > 0.001) else 0)
            angle = sum(angles) / len(angles)
            #self._logger.info(f'angle: {angle}')
            error = angle - 0.7   # !!!!!!!!!!! зависит от матрицы гомографии!!!!!!!!
            p_coef = 0.7
            command_message = AckermannDrive()
            command_message.speed = float(speed)
            command_message.steering_angle = error / math.pi * p_coef
        
        world_model.command_message= command_message
        # with open('/home/hiber/angle.csv','a') as fd:
        #     fd.write(f'{command_message.speed},{command_message.steering_angle},{datetime.now()}\n')
        #command_message.steering_angle = 0.0
        #self._logger.info(f'angle: {angle}; diff: {error * p_coef}')

    def log(self, message):
        self.node._logger.info(message)    