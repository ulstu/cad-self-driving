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


class NodeDriveGazelle(Node):
    def __init__(self):
        try:
            super().__init__('node_drive_gazelle')
            self.create_subscription(AckermannDrive, 'cmd_ackermann', self.__cmd_ackermann_callback, 1)
            self.udp_ip = "192.168.1.101"
            self.udp_port = 90
            self.sock = socket.socket(socket.AF_INET,
                                      socket.SOCK_DGRAM) 
        except Exception as err:
            self._logger.error(''.join(traceback.TracebackException.from_exception(err).format()))

    def __cmd_ackermann_callback(self, message):
        speed = message.speed
        steering_angle = message.steering_angle

        message = {
            "jsonrpc": "2.0",
            "method": "set_control_values",
            "params": [speed, steering_angle]
        }

        str_message = json.dumps(message)
        self.sock.sendto(str_message.encode(), (self.udp_ip, self.udp_port))

        self._logger.info(f'GAZELLE drive message: {speed} {steering_angle}')


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
