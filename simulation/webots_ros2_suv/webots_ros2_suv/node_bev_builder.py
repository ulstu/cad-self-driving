import os
from datetime import datetime
import rclpy
import numpy as np
import traceback
import cv2
import json
import sys
import math
import yaml
import matplotlib.pyplot as plt
import sensor_msgs.msg
import struct

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
from PIL import Image as PImage

from .lib import ipm_transformer
from .lib.timeit import timeit
from .lib.world_model import WorldModel
from .lib.orientation import euler_from_quaternion
from .lib.finite_state_machine import FiniteStateMachine
from .lib.map_server import start_web_server, MapWebServer
import threading
from sensor_msgs.msg import Image, NavSatFix, NavSatStatus, PointCloud2, PointField
from robot_interfaces.srv import PoseService
from robot_interfaces.msg import EgoPose
from .lib.param_loader import ParamLoader
import tkinter as tk
from tkinter import *

FPS = 2
PACKAGE_NAME = '/home/hiber/repositories/cad-self-driving/simulation/webots_ros2_suv'
DATACAMERA = "/home/hiber/ros2_ws/data/camera/buffer.png"
DATALIDAR = "/home/hiber/ros2_ws/data/lidar/buffer.json"

# Класс для всего узла
class NodeBEVBuilder(Node):
    def __init__(self):
        self.obstacles = []
        try:
            # Инициализация узла
            super().__init__('node_bev_builder')
            self._logger.info('Node BEV Builder Started')
            param = ParamLoader()

            qos = qos_profile_sensor_data
            qos.reliability = QoSReliabilityPolicy.RELIABLE
            self.load_ipm_config(f'{PACKAGE_NAME}/config/ipm_config.yaml')

            self.calc_points()
            self.__ipm.calc_homography(self.pts_src, self.pts_dst)
            

            # Создаём подписч который принимает JSON в виде строки и при приёме данных вызывается функция __on_obstacles_message
            self.create_subscription(sensor_msgs.msg.Image, param.get_param("front_image_topicname"), self.__on_image_message, qos)
            # Создаём подписчика, который принимает JSON в виде строки и при приёме данных вызывается функция __on_obstacles_message
            self.create_subscription(String, 'obstacles', self.__on_obstacles_message, qos)
            
            with open(f"{PACKAGE_NAME}/config/lidardata.yaml", "r") as file:
                self.lidardata = yaml.safe_load(file)
            self.MAP_SCALE = self.lidardata['visual_scale']
            self.rear_figures = []
            self.front_figures = []
            self.__last_image_time = datetime.now()
 
        except  Exception as err:
            self._logger.error(''.join(traceback.TracebackException.from_exception(err).format()))

    # Функция вызывается, когда прилетает JSON от pcl_map_node
    def __on_obstacles_message(self, data):
        obstacles_dict = json.loads(data.data);
        with open(DATALIDAR, 'w') as fp:
            json.dump({"data": obstacles_dict.get("obstacles", [])}, fp)




    def calc_points(self):
        self.pts_src[:, 1] = self.pts_src[:, 1] - self.__horizont_line_height
        self.pts_dst[:, 1] = self.pts_dst[:, 1] - self.__horizont_line_height
        
    def load_ipm_config(self, config_path):
        if not os.path.exists(config_path):
            self._logger.info(f'Config file not found. Use default values. Path: {config_path}')
            return
        with open(config_path) as file:
            config = yaml.full_load(file)
            self.__horizont_line_height = config['horizont']
            self.__img_height = config['height']
            self.__img_width = config['width']  
            self.pts_src = np.array(config['src_points'])
            self.pts_dst = np.array(config['dst_points'])
            try:
                self.posx = config['posx']
                self.posy = config['posy']
                self.scale = config['scale']
            except:
                self._logger.info(f'Position not loaded')
            self.__ipm = ipm_transformer.IPMTransformer(np.array(config['homography']))
            self._logger.info(f'Loaded')

    
    def get_ipm(self, image):
        h_img = cv2.resize(image, (self.__img_width, self.__img_height))
        return self.__ipm.get_ipm(h_img, is_mono=False,
                                                      horizont=self.__horizont_line_height) 
        # pil_img = Image.fromarray(self.__img_ipm)
        # target_width = self.__img_width  # 400
        # pil_img = pil_img.resize((target_width, int(pil_img.size[1] * target_width / pil_img.size[0])))

    def __on_image_message(self, data):
        if (datetime.now() - self.__last_image_time).total_seconds() < 1 / FPS:
                return
        self.__last_image_time = datetime.now()
        image = data.data
        image = np.frombuffer(image, dtype=np.uint8).reshape((data.height, data.width, 4))
        analyze_image = cv2.cvtColor(image, cv2.COLOR_RGBA2RGB)
        cv2.imwrite(DATACAMERA, analyze_image)
        # self.ia.load_image(image)
        
# Запуск и работа всей ноды
def main(args=None):
    try:
        rclpy.init(args=args) 
        bevbuilder_controller = NodeBEVBuilder()
        rclpy.spin(bevbuilder_controller)
        rclpy.shutdown()
    except KeyboardInterrupt:
        print('server stopped cleanly')
    except  Exception as err:
        print(''.join(traceback.TracebackException.from_exception(err).format()))
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()