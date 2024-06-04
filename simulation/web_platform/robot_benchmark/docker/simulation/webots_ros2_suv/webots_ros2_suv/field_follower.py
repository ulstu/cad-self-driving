import os
import cv2
import numpy as np
import rclpy
import pathlib
import math
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
import traceback



CONTROL_COEFFICIENT = 0.0007
CONTROL_COEFFICIENT_DETOUR = 0.3183
ANGLE_GAP = 0.435808714

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
            #self.create_subscription(Image, 'vehicle/camera', self.__on_camera_image, qos_camera_data)
            self._logger.info('Field path follower initialized')
            package_dir = get_package_share_directory('webots_ros2_suv')
            points_path = f'{package_dir}/worlds/ulstu_field_points.txt'
            points = open(points_path, 'r')
            self.current_x = 210.23121027069885
            self.current_y = 77.42130129912289
            self.current_angle = 0
            self.x_coordinates = []
            self.y_coordinates = []
            x_flag = 1
            self.index_next_point = 1
            with open(points_path, 'r') as file:
                for line in file:
                    if x_flag == 1:
                        self.x_coordinates.append(float(line.strip()))
                        x_flag = 0
                    else:
                        self.y_coordinates.append(float(line.strip()))
                        x_flag = 1

            self._logger.info(points.read(500))
        except  Exception as ex:
            self._logger.error(''.join(traceback.TracebackException.from_exception(ex).format()))

    #def wheel_control(self):
        #try:
            #infinity = 1
            #while infinity:
              #  self._logger.info(str(self.current_angle))

            #else:
              #  end = True
        #except  Exception as ex:
          #  self._logger.error(''.join(traceback.TracebackException.from_exception(ex).format()))


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

        return roll_x, pitch_y, yaw_z  # in radians

    def __on_odom(self, message):
        try:
            #self._logger.info(f'odom x: {message.pose.pose.position.x} y: {message.pose.pose.position.y} z: {message.pose.pose.position.z}')
            (roll, pitch, yaw) = self.euler_from_quaternion(message.pose.pose.orientation.x,
                                                            message.pose.pose.orientation.y,
                                                            message.pose.pose.orientation.z,
                                                            message.pose.pose.orientation.w)
            # self._logger.info(f'yaw: {yaw}')
            self.current_angle = yaw - ANGLE_GAP
            self.current_x = float(message.pose.pose.position.x)
            self.current_y = float(message.pose.pose.position.y)
            if len(self.x_coordinates) != self.index_next_point:
                delta = self.calculate_distance(self.current_x, self.current_y,
                                                self.x_coordinates[self.index_next_point],
                                                self.y_coordinates[self.index_next_point])

                if (delta <= 3):
                    self.index_next_point += 1
                    self._logger.info(str(self.index_next_point))


            command_message = AckermannDrive()
            command_message.speed = 2.0
            command_message.steering_angle = 0.0
            calc_angle = self.calculate_angle(self.current_x, self.current_y,
                                                              self.x_coordinates[self.index_next_point],
                                                              self.y_coordinates[self.index_next_point])
            error = self.current_angle - calc_angle
            if error > 3.14159:
                error -= 6,28319
            elif error < 3.14159:
                error -= 6, 28319
            #self._logger.info(f'yaw: {yaw:.5f} curangle" {self.current_angle:.5f} calc angle: {calc_angle:.5f}')
            command_message.steering_angle = error * CONTROL_COEFFICIENT_DETOUR
            set_transmission(command_message.speed / 25 + 1)
            set_steering(command_message.steering_angle)

            self.__ackermann_publisher.publish(command_message)
        except  Exception as ex:
            self._logger.error(''.join(traceback.TracebackException.from_exception(ex).format()))

    def calculate_angle(self, x1, y1, x2, y2):
        dx = x2 - x1
        dy = y2 - y1
        angle = math.atan2(dy, dx)  # Calculate the angle in radians
        #angle_deg = math.degrees(angle)  # Convert the angle to degrees
        return angle

    def calculate_distance(self, x1, y1, x2, y2):
        distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        return distance

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
        except  Exception as ex:
            self._logger.error(''.join(traceback.TracebackException.from_exception(ex).format()))


def main(args=None):
    try:
        rclpy.init(args=args)
        follower = LaneFollower()
        #follower.wheel_control()
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
