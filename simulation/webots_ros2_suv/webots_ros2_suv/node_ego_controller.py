import rclpy
import numpy as np
import traceback
import cv2
import math
import matplotlib.pyplot as plt
import sensor_msgs.msg
#from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String
from nav_msgs.msg import Odometry
from geometry_msgs.msg  import PointStamped, TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
from ackermann_msgs.msg import AckermannDrive
from rclpy.qos import qos_profile_sensor_data, QoSReliabilityPolicy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.utils import controller_url_prefix
from PIL import Image
from .lib.world_model import WorldModel
from .lib.orientation import euler_from_quaternion
from .lib.finite_state_machine import FiniteStateMachine

from robot_interfaces.srv import PoseService
from robot_interfaces.msg import EgoPose

SENSOR_DEPTH = 40

class NodeEgoController(Node):
    def __init__(self):
        try:
            super().__init__('node_ego_controller')
            self._logger.info(f'Node Ego Started')
            qos = qos_profile_sensor_data
            qos.reliability = QoSReliabilityPolicy.RELIABLE
            self.create_subscription(Odometry, '/odom', self.__on_gps_message, qos)
            self.create_subscription(sensor_msgs.msg.Image, '/vehicle/camera/image_color', self.__on_image_message, qos)
            #self.create_subscription(sensor_msgs.msg.Image, '/vehicle/left_wing_camera/image_color', self.__on_left_image_message, qos)
            self.create_subscription(sensor_msgs.msg.Image, '/vehicle/range_finder/image', self.__on_range_image_message, qos)

            self.__ackermann_publisher = self.create_publisher(AckermannDrive, 'cmd_ackermann', 1)

            self.__world_model = WorldModel()
            self.srv = self.create_service(PoseService, 'get_current_global_pos', self.get_current_global_pos)

            package_dir = get_package_share_directory("webots_ros2_suv")
            self.__fsm = FiniteStateMachine(f'{package_dir}/config/ego_states/robocross.yaml', self)
            # Примеры событий
            self.__fsm.on_event("start_move")
            # self.__fsm.on_event("stop")
            # self.__fsm.on_event("reset")

        except  Exception as err:
            self._logger.error(''.join(traceback.TracebackException.from_exception(err).format()))
    
    def __on_range_image_message(self, data):
        image = np.frombuffer(data.data, dtype="float32").reshape((data.height, data.width, 1))
        image[image == np.inf] = SENSOR_DEPTH
        image[image == -np.inf] = 0
        self.__world_model.range_image = image / SENSOR_DEPTH
        #self.__world_model = self.__fsm.on_data(self.__world_model, source="__on_range_image_message")

    def drive(self):
        if (len(self.__world_model.path) < 3):
            command_message = AckermannDrive()
            command_message.speed = 0.0
            command_message.steering_angle = 0.0
            self.__ackermann_publisher.publish(command_message)
            return
        p1, p2 = self.__world_model.path[0], self.__world_model.path[3]
        angle = math.asin((p2[0] - p1[0]) / math.sqrt((p2[1] - p1[1]) ** 2 + (p2[0] - p1[0]) ** 2))
        #self._logger.info(f'angle: {angle}')
        error = angle - 0.7   # !!!!!!!!!!! зависит от матрицы гомографии!!!!!!!!
        p_coef = 0.7
        command_message = AckermannDrive()
        command_message.speed = 8.0
        command_message.steering_angle = error / math.pi * p_coef

        # with open('/home/hiber/angle.csv','a') as fd:
        #     fd.write(f'{command_message.speed},{command_message.steering_angle},{datetime.now()}\n')
        #command_message.steering_angle = 0.0
        #self._logger.info(f'angle: {angle}; diff: {error * p_coef}')
        self.__ackermann_publisher.publish(command_message)

    def __on_image_message(self, data):
        image = data.data
        image = np.frombuffer(image, dtype=np.uint8).reshape((data.height, data.width, 4))
        analyze_image = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_RGBA2RGB))

        self.__world_model.rgb_image = np.asarray(analyze_image)
        self._logger.info("image received")
        self.__world_model = self.__fsm.on_data(self.__world_model, source="__on_image_message")
        self.drive()

    def __on_gps_message(self, data):
        roll, pitch, yaw = euler_from_quaternion(data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
        lat, lon, orientation = self.__world_model.get_global_coords(data.pose.pose.position.x, data.pose.pose.position.y, yaw)
        #self._logger.info(f'transformed lat: {lat}; lon: {lon}; orientation: {orientation}')

    def get_current_global_pos(self, request, response):
        self.get_logger().info('Incoming request position')
        lat, lon, orientation = self.__world_model.get_current_position()
        response.response.lat = float(lat)
        response.response.lon = float(lon)
        response.response.orientation = float(orientation)
        self._logger.info(f'transformed lat: {lat}; lon: {lon}; orientation: {orientation}')
        return response

def main(args=None):
    try:
        rclpy.init(args=args)
        path_controller = NodeEgoController()
        rclpy.spin(path_controller)
        rclpy.shutdown()
    except KeyboardInterrupt:
        print('server stopped cleanly')
    except  Exception as err:
        print(''.join(traceback.TracebackException.from_exception(err).format()))
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
