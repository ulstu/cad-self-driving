import rclpy
import math
import numpy as np
import cv2
from sensor_msgs.msg import Image, NavSatFix, NavSatStatus, PointCloud2, Imu
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg  import PointStamped, TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
from ackermann_msgs.msg import AckermannDrive
from rclpy.qos import qos_profile_sensor_data, QoSReliabilityPolicy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from .lib.orientation import quaternion_from_euler, euler_from_quaternion
import traceback
from webots_ros2_driver.utils import controller_url_prefix


class NodeSensorsWebots(Node):
    def __init__(self):
        try:
            self.__vehicles = ['vehicle', 'vehicle1']
            super().__init__('node_gps')
            self._logger.info(controller_url_prefix())

            qos = qos_profile_sensor_data
            qos.reliability = QoSReliabilityPolicy.RELIABLE
            self.__gps_publishers = {}
            self.__odom_publishers = {}
            self.__pc_publishers = {}
            self.__cur_imu_datas = {}
            for vehicle in self.__vehicles:
                self._logger.info(f'VEHICLE INIT: {vehicle}')
                self.__gps_publishers[vehicle] = self.create_publisher(NavSatFix, f"/{vehicle}/gps_nav", qos)
                self.__odom_publishers[vehicle] = self.create_publisher(Odometry, f"/{vehicle}/odom", qos)
                self.__pc_publishers[vehicle] = self.create_publisher(PointCloud2, f"/{vehicle}/lidar", qos)

                self.create_subscription(PointStamped, f'/{vehicle}/gps', lambda msg, v = vehicle: self.__on_gps_message(msg, v), qos_profile=qos)
                self.create_subscription(Image, f'/{vehicle}/range_finder', lambda msg, v = vehicle: self.__on_range_message(msg, v), qos_profile=qos)
                self.create_subscription(Float32, f'/{vehicle}/gps/speed', lambda msg, v = vehicle: self.__on_speed(msg, v), qos_profile=qos)
                self.create_subscription(PointCloud2, f'/{vehicle}/lidar_base/point_cloud', lambda msg, v = vehicle: self.__on_point_cloud(msg, v) , qos_profile=qos)
                self.create_subscription(Imu, f'/{vehicle}/imu', lambda msg, v = vehicle: self.__on_imu(msg, v), qos_profile=qos)

            self.__tf_broadcaster = TransformBroadcaster(self)

            self._logger.info('GPS Node initialized')
            self.__cur_speed = 0.0
            self.__lidar_last_time = self.get_clock().now()
            self.__gnss_last_time = self.get_clock().now()
            self.__camera_last_time = self.get_clock().now()
        except  Exception as err:
            print(f'{str(err)}')


    def set_last_time(self, last_time, set_func):
        cur_time = self.get_clock().now()

        time_diff = (cur_time - last_time).nanoseconds / 1e9
        if time_diff == 0.0:
            time_diff = 0.0
        else:
            time_diff = 1 / time_diff
        set_func(time_diff)
        return cur_time

    def __on_speed(self, data, vehicle):
        self.__cur_speed = float(data.data)

    def __on_range_message(self, data, vehicle):
        try:
            pass
        except  Exception as err:
            print(f'{str(err)}')
    
    def __on_imu(self, data, vehicle):
        self.__cur_imu_datas[vehicle] = data

    def __on_point_cloud(self, data, vehicle):
        try:
            p = data
            p.header.stamp = self.get_clock().now().to_msg()
            p.header.frame_id = f'base_link_{vehicle}'
            self.__pc_publishers[vehicle].publish(p)
        except  Exception as err:
            self._logger.error(''.join(traceback.TracebackException.from_exception(err).format()))
            

    def __on_gps_message(self, data, vehicle):
        try:
            if not vehicle in self.__cur_imu_datas:
                self._logger.error('NO IMU DATA!')
                return 
            (roll, pitch, yaw) = euler_from_quaternion(self.__cur_imu_datas[vehicle].orientation.x, self.__cur_imu_datas[vehicle].orientation.y, self.__cur_imu_datas[vehicle].orientation.z, self.__cur_imu_datas[vehicle].orientation.w)
            stamp = self.get_clock().now().to_msg()
            msg = NavSatFix()
            msg.header.stamp = stamp
            msg.header.frame_id = f'base_link_{vehicle}'
            msg.latitude = data.point.x
            msg.longitude = data.point.y
            msg.altitude = data.point.z
            msg.status.status = NavSatStatus.STATUS_SBAS_FIX
            msg.status.service = NavSatStatus.SERVICE_GPS | NavSatStatus.SERVICE_GLONASS | NavSatStatus.SERVICE_COMPASS | NavSatStatus.SERVICE_GALILEO
            self.__gps_publishers[vehicle].publish(msg) 


            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = f'odom_{vehicle}'
            t.child_frame_id = f'base_link_{vehicle}'
            t.transform.translation.x = data.point.x
            t.transform.translation.y = data.point.y
            t.transform.translation.z = data.point.z
            t.transform.rotation = self.__cur_imu_datas[vehicle].orientation

            self.__tf_broadcaster.sendTransform(t)

            odom = Odometry()
            odom.header.frame_id = f'odom_{vehicle}'
            odom.header.stamp = stamp
            odom.child_frame_id = f'base_link_{vehicle}'
            odom.pose.pose.position.x = data.point.x
            odom.pose.pose.position.y = data.point.y
            odom.pose.pose.position.z = data.point.z
            odom.pose.pose.orientation = self.__cur_imu_datas[vehicle].orientation
            self.__odom_publishers[vehicle].publish(odom)
        except  Exception as err:
            self._logger.error(''.join(traceback.TracebackException.from_exception(err).format()))


def main(args=None):
    try:
        rclpy.init(args=args)
        detector = NodeSensorsWebots()
        rclpy.spin(detector)
        rclpy.shutdown()
    except KeyboardInterrupt:
        print('server stopped cleanly')
    except  Exception as err:
        print(''.join(traceback.TracebackException.from_exception(err).format()))
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
