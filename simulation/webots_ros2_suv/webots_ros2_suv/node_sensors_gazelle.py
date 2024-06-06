import math
import rclpy
import traceback
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


class NodeSensorsGazelle(Node):
    def __init__(self):
        try:
            super().__init__('node_sensors_gazelle')
            self.get_logger().info(f'STARTING NODE SENSORS GAZELLE: {controller_url_prefix()}')

            qos = qos_profile_sensor_data
            qos.reliability = QoSReliabilityPolicy.RELIABLE
            param = ParamLoader()
            self.__gps_publisher = self.create_publisher(NavSatFix, param.get_param("navsatfix_topicname"), qos)
            self.__odom_publisher = self.create_publisher(Odometry, param.get_param("odom_topicname"), qos)
            self.__tf_broadcaster = TransformBroadcaster(self)
            
            self.__gps = GPSReader(param.get_param("gnss_port"), int(param.get_param("gnss_baudrate")))
            self.get_logger().info('Gazelle GPS Node initialized and timer started')
        except Exception as err:
            self._logger.error(''.join(traceback.TracebackException.from_exception(err).format()))

    def gps_read(self):
        try:
            #self.get_logger().info('BEFORE GPS READ')
            self.__gps.read_data()
            #self.get_logger().info(f'GPS DATA: lat: {self.__gps.lat_dec}, lon: {self.__gps.lon_dec} yaw: {self.__gps.yaw}')
            yaw = self.__gps.yaw
            stamp = self.get_clock().now().to_msg()
            msg = NavSatFix()
            msg.header.stamp = stamp
            msg.header.frame_id = 'base_link'
            msg.latitude = self.__gps.lon_dec
            msg.longitude = self.__gps.lat_dec
            msg.altitude = self.__gps.altitude
            msg.status.status = NavSatStatus.STATUS_SBAS_FIX
            msg.status.service = NavSatStatus.SERVICE_GPS | NavSatStatus.SERVICE_GLONASS | NavSatStatus.SERVICE_COMPASS | NavSatStatus.SERVICE_GALILEO
            self.__gps_publisher.publish(msg)         

            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = self.__gps.lon_dec
            t.transform.translation.y = self.__gps.lat_dec
            t.transform.translation.z = self.__gps.altitude
            t.transform.rotation = quaternion_from_euler(0.0, 0.0, yaw, ros_quaternion=True)

            self.__tf_broadcaster.sendTransform(t)

            odom = Odometry()
            odom.header.frame_id = "odom"
            odom.header.stamp = stamp
            odom.child_frame_id = "base_link"
            odom.pose.pose.position.x = self.__gps.lon_dec
            odom.pose.pose.position.y = self.__gps.lat_dec
            odom.pose.pose.position.z = self.__gps.altitude
            odom.pose.pose.orientation = quaternion_from_euler(0.0, 0.0, yaw, ros_quaternion=True)
            self.__odom_publisher.publish(odom)
            set_location(self.__gps.velocity, self.__gps.lat_dec, self.__gps.lon_dec, self.__gps.altitude, yaw)
        except Exception as err:
            self._logger.error(''.join(traceback.TracebackException.from_exception(err).format()))


def main(args=None):
    try:
        start_http_server(8009)
    except Exception as err:
        print(''.join(traceback.TracebackException.from_exception(err).format()))
    try:
        set_state(LogServerStatus.STARTED)
        rclpy.init(args=args)
        set_state(LogServerStatus.RUNNING)
        detector = NodeSensorsGazelle()
        while rclpy.ok():
            try:
                rclpy.spin_once(detector, timeout_sec=0.1)
                detector.gps_read()
            except KeyboardInterrupt:
                pass
        detector.destroy_node()
        set_state(LogServerStatus.STOPPED)
        rclpy.shutdown()
    except KeyboardInterrupt:
        set_state(LogServerStatus.STOPPED)
        print('server stopped cleanly')
    except Exception as err:
        set_state(LogServerStatus.STOPPED)
        print(''.join(traceback.TracebackException.from_exception(err).format()))
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
