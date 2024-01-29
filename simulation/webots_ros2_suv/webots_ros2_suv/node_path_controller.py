import rclpy
import numpy as np
import traceback
from sensor_msgs.msg import Image, NavSatFix, NavSatStatus, PointCloud2, Imu
from std_msgs.msg import Float32, String
from nav_msgs.msg import Odometry
from geometry_msgs.msg  import PointStamped, TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
from ackermann_msgs.msg import AckermannDrive
from rclpy.qos import qos_profile_sensor_data, QoSReliabilityPolicy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.utils import controller_url_prefix
from .lib.world_model import WorldModel
from .lib.orientation import euler_from_quaternion
from .lib.finite_state_machine import FiniteStateMachine

from robot_interfaces.srv import PoseService
from robot_interfaces.msg import EgoPose


class NodePathController(Node):
    def __init__(self):
        try:
            super().__init__('node_path_controller')
            self._logger.info(f'Node Path Started')
            qos = qos_profile_sensor_data
            qos.reliability = QoSReliabilityPolicy.RELIABLE
            self.create_subscription(Odometry, '/odom', self.__on_gps_message, qos)
            self.__world_model = WorldModel()
            self.srv = self.create_service(PoseService, 'get_current_global_pos', self.get_current_global_pos)

            package_dir = get_package_share_directory("webots_ros2_suv")
            self.__fsm = FiniteStateMachine(f'{package_dir}/config/ego_states/robocross.yaml', self)
            # Примеры событий
            self.__fsm.on_event("start_move")
            self.__fsm.on_event("stop")
            self.__fsm.on_event("reset")

        except  Exception as err:
            self._logger.error(''.join(traceback.TracebackException.from_exception(err).format()))

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
        path_controller = NodePathController()
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
