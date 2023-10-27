import os
import cv2
import numpy as np
import rclpy
import pathlib
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDrive
from rclpy.qos import qos_profile_sensor_data, QoSReliabilityPolicy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSReliabilityPolicy
from  .log_server import set_transmission
from  .log_server import set_steering
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_share_directory
from launch.substitutions.path_join_substitution import PathJoinSubstitution




CONTROL_COEFFICIENT = 0.0007
class LaneFollower(Node):
    def __init__(self):
        try:
            super().__init__('field_follower')

            # ROS interface
            self.__ackermann_publisher = self.create_publisher(AckermannDrive, 'cmd_ackermann', 1)
            qos = qos_profile_sensor_data
            qos.reliability = QoSReliabilityPolicy.RELIABLE
            self.create_subscription(Odometry, '/odom', self.__on_odom, qos)

            qos_camera_data = qos_profile_sensor_data
            # In case ROS_DISTRO is not foxy the QoSReliabilityPolicy is strict.
            if 'ROS_DISTRO' in os.environ and os.environ['ROS_DISTRO'] != 'foxy':
                qos_camera_data.reliability = QoSReliabilityPolicy.RELIABLE
            self.create_subscription(Image, 'vehicle/camera', self.__on_camera_image, qos_camera_data)
            self._logger.info('Field path follower initialized')
            package_dir = get_package_share_directory('webots_ros2_suv')
            points_path = f'{package_dir}/worlds/ulstu_field_points.txt'
            points = open(points_path, 'r')

            self._logger.info(points.read(500))
        except  Exception as err:
            self._logger.error(f'{str(err)}')

    def __on_odom(self, message):
        self._logger.info(f'odom x: {message.pose.pose.position.x} y: {message.pose.pose.position.y} z: {message.pose.pose.position.z}')

    def __on_camera_image(self, message):
        try:
            img = message.data
            img = np.frombuffer(img, dtype=np.uint8).reshape((message.height, message.width, 4))
            img = img[120:240, :]

            # Segment the image by color in HSV color space
            img = cv2.cvtColor(img, cv2.COLOR_RGBA2RGB)
            #img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
            #mask = cv2.inRange(img, np.array([50, 110, 150]), np.array([120, 255, 255]))
            mask = cv2.inRange(img, np.array([220, 220, 220]), np.array([255, 255, 255]))

            # Find the largest segmented contour
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

            command_message = AckermannDrive()
            command_message.speed = 2.0
            command_message.steering_angle = 0.0

            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                largest_contour_center = cv2.moments(largest_contour)

                cv2.drawContours(img, largest_contour, -1, (0,255,0), 3)
                cv2.imshow("img", img)
                if cv2.waitKey(25) & 0xFF == ord('q'):
                    return
                
                if largest_contour_center['m00'] != 0:
                    center_x = int(largest_contour_center['m10'] / largest_contour_center['m00'])
                    error = 190 - center_x
                    command_message.steering_angle = error * CONTROL_COEFFICIENT

            set_transmission(command_message.speed / 25 + 1)
            set_steering(command_message.steering_angle)

            self.__ackermann_publisher.publish(command_message)
        except  Exception as err:
            self._logger.info(f'{str(err)}')

def main(args=None):
    try:
        rclpy.init(args=args)
        follower = LaneFollower()
        rclpy.spin(follower)
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    except  Exception as err:
        print(f'node_gps stopped')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
