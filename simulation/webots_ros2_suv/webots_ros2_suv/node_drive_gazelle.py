from std_msgs.msg import Bool
import rclpy
import traceback
import json
import threading
from sensor_msgs.msg import NavSatFix, NavSatStatus
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import qos_profile_sensor_data, QoSReliabilityPolicy
from rclpy.node import Node
from prometheus_client import start_http_server
from .log_server import set_location
from .log_server import set_state
from .log_server import LogServerStatus
from .lib.gps_reader import GPSReader
from .lib.param_loader import ParamLoader
from .lib.orientation import quaternion_from_euler
from webots_ros2_driver.utils import controller_url_prefix
from ackermann_msgs.msg import AckermannDrive
import socket

UDP_SEND_IP = '192.168.1.101'
UDP_SEND_PORT = 90


class NodeDriveGazelle(Node):
    def __init__(self):
        try:
            super().__init__('node_drive_gazelle')

            self.create_subscription(AckermannDrive, 'cmd_ackermann', self.__cmd_ackermann_callback, 1)
            self.create_subscription(Bool, 'cmd_control_unit', self.__cmd_control_unit_callback, 1)

            self.socket_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        except Exception as err:
            self._logger.error(''.join(traceback.TracebackException.from_exception(err).format()))

    def __cmd_ackermann_callback(self, message):
        speed = message.speed
        steering_angle = message.steering_angle

        jsonrpc_message = {
            "jsonrpc": "2.0",
            "method": "set_control_values",
            "params": [speed, steering_angle]
        }

        jsonrpc_message_str = json.dumps(jsonrpc_message)
        self.socket_send.sendto(jsonrpc_message_str.encode(), (UDP_SEND_IP, UDP_SEND_PORT))

        self._logger.info(f'GAZELLE drive message: {speed} {steering_angle}')

    def __cmd_control_unit_callback(self, is_pause):
        jsonrpc_message = {
            "jsonrpc": "2.0",
            "method": "set_pause_state",
            "params": [is_pause.data]
        }

        jsonrpc_message_str = json.dumps(jsonrpc_message)
        self.socket_send.sendto(jsonrpc_message_str.encode(), (UDP_SEND_IP, UDP_SEND_PORT))


def main(args=None):
    try:
        rclpy.init(args=args)
        gazelle_driver = NodeDriveGazelle()
        rclpy.spin(gazelle_driver)
        gazelle_driver.destroy_node()
    except Exception as err:
        print(''.join(traceback.TracebackException.from_exception(err).format()))
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
