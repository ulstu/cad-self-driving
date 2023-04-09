import rclpy
import math
import numpy as np
import cv2
from sensor_msgs.msg import Image, NavSatFix, NavSatStatus, PointCloud2, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg  import PointStamped, TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
from ackermann_msgs.msg import AckermannDrive
from rclpy.qos import qos_profile_sensor_data, QoSReliabilityPolicy
from rclpy.node import Node
from scipy.spatial.transform import Rotation

class NodeGPS(Node):
    def __init__(self):
        try:
            super().__init__('node_gps')
            qos = qos_profile_sensor_data
            qos.reliability = QoSReliabilityPolicy.RELIABLE
            self.__gps_publisher = self.create_publisher(NavSatFix, "/vehicle/gps_nav", qos)
            self.__odom_publisher = self.create_publisher(Odometry, "/odom", qos)
            self.__pc_publisher = self.create_publisher(PointCloud2, '/lidar', qos)
            self.create_subscription(PointStamped, '/vehicle/gps', self.__on_gps_message, qos)
            self.create_subscription(Image, '/vehicle/range_finder', self.__on_range_message, qos)
            self.create_subscription(PointCloud2, '/vehicle/Velodyne_VLP_16/point_cloud', self.__on_point_cloud, qos)
            self.create_subscription(Imu, '/imu', self.__on_imu, qos)
            self.__cur_imu_data = None
            self.__tf_broadcaster = TransformBroadcaster(self)
            self._logger.info('GPS Node initialized')
        except  Exception as err:
            print(f'{str(err)}')

    def __on_range_message(self, data):
        try:
            pass
            # img = data.data
            # img = np.frombuffer(img, dtype=np.uint8).reshape((data.height, data.width, 4))
            # cv2.imshow("range", img)
            # if cv2.waitKey(25) & 0xFF == ord('q'):
            #     return
        except  Exception as err:
            print(f'{str(err)}')
    
    def __on_imu(self, data):
        self.__cur_imu_data = data

    
    def __on_point_cloud(self, data):
        try:
            p = data
            p.header.stamp = self.get_clock().now().to_msg()
            p.header.frame_id = 'base_link'
            self.__pc_publisher.publish(p)
        except  Exception as err:
            print(f'{str(err)}')
            
    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Converts euler roll, pitch, yaw to quaternion (w in last place)
        quat = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = cy * cp * cr + sy * sp * sr
        q[1] = cy * cp * sr - sy * sp * cr
        q[2] = sy * cp * sr + cy * sp * cr
        q[3] = sy * cp * cr - cy * sp * sr

        return q

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians


    
    def __on_gps_message(self, data):
        try:
            if not self.__cur_imu_data:
                return 

            (roll, pitch, yaw) = self.euler_from_quaternion(self.__cur_imu_data.orientation.x, self.__cur_imu_data.orientation.y, self.__cur_imu_data.orientation.z, self.__cur_imu_data.orientation.w)
            #self._logger.info(f'direction: {yaw}')
            stamp = self.get_clock().now().to_msg()
            msg = NavSatFix()
            msg.header.stamp = stamp
            msg.header.frame_id = 'base_link'
            msg.latitude = data.point.x
            msg.longitude = data.point.y
            msg.altitude = data.point.z
            msg.status.status = NavSatStatus.STATUS_SBAS_FIX
            msg.status.service = NavSatStatus.SERVICE_GPS | NavSatStatus.SERVICE_GLONASS | NavSatStatus.SERVICE_COMPASS | NavSatStatus.SERVICE_GALILEO
            self.__gps_publisher.publish(msg) 


            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = data.point.x
            t.transform.translation.y = data.point.y
            t.transform.translation.z = data.point.z
            t.transform.rotation = self.__cur_imu_data.orientation

            self.__tf_broadcaster.sendTransform(t)

            odom = Odometry()
            odom.header.frame_id = "odom"
            odom.header.stamp = stamp
            odom.child_frame_id = "base_link"
            odom.pose.pose.position.x = data.point.x
            odom.pose.pose.position.y = data.point.y
            odom.pose.pose.position.z = data.point.z
            odom.pose.pose.orientation = self.__cur_imu_data.orientation
            self.__odom_publisher.publish(odom)
        except  Exception as err:
            print(f'{str(err)}')


def main(args=None):
    rclpy.init(args=args)
    detector = NodeGPS()
    rclpy.spin(detector)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
