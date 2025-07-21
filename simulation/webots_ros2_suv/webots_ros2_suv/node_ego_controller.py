import rclpy
import numpy as np
import traceback
import cv2
import os
import math
import time
import yaml
import threading
import matplotlib.pyplot as plt
import sensor_msgs.msg
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
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
from .lib.timeit import timeit
from .lib.orientation import euler_from_quaternion
from .lib.map_server import start_web_server, MapWebServer
from .lib.world_model import WorldModel
from robot_interfaces.srv import PoseService
from robot_interfaces.msg import EgoPose
from datetime import timedelta
from builtin_interfaces.msg import Time
from rclpy.time import Time as RclpyTime

SENSOR_DEPTH = 40

class NodeEgoController(Node):
    def __init__(self):
        try:
            super().__init__('node_ego_controller')
            self._logger.info(f'Node Ego Started')
            qos = qos_profile_sensor_data
            qos.reliability = QoSReliabilityPolicy.RELIABLE

            self.__vehicles = ['vehicle', 'vehicle1']
            self.__world_model = WorldModel(self.__vehicles)
            self.__ws = None
            
            package_dir = get_package_share_directory("webots_ros2_suv")
            self.__ackermann_publishers = {}
            self.__local_coords = {}
            for vehicle in self.__vehicles:
                self.create_subscription(Odometry, f'/{vehicle}/odom', lambda msg, v = vehicle: self.__on_odom_message(msg, v), qos)
                self.create_subscription(sensor_msgs.msg.Image, f'/{vehicle}/camera/image_color', lambda msg, v = vehicle: self.__on_image_message(msg, v), qos)
                self.create_subscription(sensor_msgs.msg.PointCloud2, f"/{vehicle}/lidar", lambda msg, v = vehicle: self.__on_lidar_message(msg, v), qos)

                self.__ackermann_publishers[vehicle] = self.create_publisher(AckermannDrive, f'/{vehicle}/cmd_ackermann', 1)
            
            # Таймер для периодического выполнения
            timer_period = 0.2  # Период в секундах
            self.start_time = self.get_clock().now()
            self.start_web_server()

        except  Exception as err:
            self._logger.error(''.join(traceback.TracebackException.from_exception(err).format()))

    def stitch_images(self, images):
        """Склеивание изображений по горизонтали."""
        # Убедимся, что все изображения имеют одинаковую высоту
        max_height = max(img.shape[0] for img in images)
        resized_images = [
            cv2.resize(img, (int(img.shape[1] * max_height / img.shape[0]), max_height))
            for img in images
        ]
        # Склеивание изображений
        return cv2.hconcat(resized_images)
    
    def start_web_server(self):
        self.__ws = MapWebServer(log=self._logger.info)
        threading.Thread(target=start_web_server, args=[self.__ws]).start()

    def __on_lidar_message(self, data, vehicle):
        pass

    def __on_range_image_message(self, data):
        image = np.frombuffer(data.data, dtype="float32").reshape((data.height, data.width, 1))
        image[image == np.inf] = SENSOR_DEPTH
        image[image == -np.inf] = 0
        
        range_image = image / SENSOR_DEPTH

    def drive(self, vehicle):
        dmsg = AckermannDrive()
        dmsg.speed = 10.0 if vehicle == "vehicle" else 8.0 # для разных автомобилей разная скорость
        dmsg.steering_angle = 0.0

        self.__ackermann_publishers[vehicle].publish(dmsg)

    #@timeit
    def __on_image_message(self, data, vehicle):
        image = data.data
        image = np.frombuffer(image, dtype=np.uint8).reshape((data.height, data.width, 4))
        analyze_image = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_RGBA2RGB))

        t1 = time.time()
        # TODO: put your code here
        t2 = time.time()
        
        delta = t2 - t1
        fps = 1 / delta if delta > 0 else 100
        # self._logger.info(f"Current FPS: {fps}")

        pos = self.__world_model.get_current_position(vehicle)
        self.__world_model.set_rgb_image(np.asarray(analyze_image), vehicle)


        self.drive(vehicle)

        if self.__ws is not None:
            self.__ws.update_model(self.__world_model)

    def __on_odom_message(self, data, vehicle):
            roll, pitch, yaw = euler_from_quaternion(data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
            lat, lon, orientation = self.__world_model.coords_transformer.get_global_coords(data.pose.pose.position.x, data.pose.pose.position.y, yaw)
            self.__local_coords[vehicle] = (data.pose.pose.position.x, data.pose.pose.position.y, yaw)
            self.__world_model.update_car_pos(lat, lon, orientation, vehicle)
            if self.__ws is not None:
                self.__ws.update_model(self.__world_model)

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
