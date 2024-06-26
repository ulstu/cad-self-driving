import os
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
from .lib.config_loader import ConfigLoader
import tkinter as tk
from tkinter import *


PACKAGE_NAME = '/home/hiber/repositories/cad-self-driving/simulation/webots_ros2_suv'

# Класс для всего узла
class NodeBEVBuilder1(Node):
    def __init__(self):
        self.ipm_img = np.zeros(shape=(1, 1, 3), dtype=np.uint8)
        self.posx = 0
        self.posy = 0
        self.scale = 1
        try:
            # Инициализация узла
            super().__init__('node_bev_builder')
            self._logger.info('Node BEV Builder Started')
            param = ParamLoader()

            qos = qos_profile_sensor_data
            qos.reliability = QoSReliabilityPolicy.RELIABLE
            self.load_ipm_config()

            self.calc_points()
            self.__ipm.calc_homography(self.pts_src, self.pts_dst)
            

            # Создаём подписчика, который принимает JSON в виде строки и при приёме данных вызывается функция __on_obstacles_message
            #self.create_subscription(String, 'obstacles', self.__on_obstacles_message, qos)
            # self._logger.info(f'topic name {param.get_param("front_image_topicname")}')
            self.create_subscription(sensor_msgs.msg.Image, param.get_param("front_image_topicname"), self.__on_image_message, qos)
            # Создаём подписчика, который принимает JSON в виде строки и при приёме данных вызывается функция __on_obstacles_message
            self.create_subscription(String, 'obstacles', self.__on_obstacles_message, qos)
            with open(f"{PACKAGE_NAME}/config/lidardata.yaml", "r") as file:
                self.lidardata = yaml.safe_load(file)
            self.MAP_SCALE = self.lidardata['visual_scale']
            self.rear_figures = []
            self.front_figures = []
 
        except  Exception as err:
            self._logger.error(''.join(traceback.TracebackException.from_exception(err).format()))

    # Функция вызывается, когда прилетает JSON от pcl_map_node
    def __on_obstacles_message(self, data):
        # self._logger.info('Node BEV Builder Continue')
        # в data.data находится наша строка, парсим её
        obstacles_dict = json.loads(data.data);
        # если прилетели данные от переднего лидара
        if 'obstacles' in obstacles_dict:
            self.front_figures = []
            obst_list = obstacles_dict['obstacles'];
        # если прилетели данные от заднего лидара
        if 'obstacles_rear' in obstacles_dict:
            self.rear_figures = []
            obst_list = obstacles_dict['obstacles_rear'];

        # Обходим все обнаруженные препятствия
        for p in obst_list:
 
                if 'obstacles' in obstacles_dict:
                    self.front_figures.append([p[4], p[5], p[6], p[7]])
                else:
                    self.rear_figures.append([p[4], p[5], p[6], p[7]])

        

        self.all_figures = self.front_figures + self.rear_figures

        # Создаём черный фон
        self.img = np.zeros(shape=(800, 800, 4), dtype=np.uint8)
        self.img_full = np.zeros(shape=(800, 800, 3), dtype=np.uint8)
       



        scale = self.MAP_SCALE # Домножаем на scale, чтобы картинка не была слишком мелкой, т.к. координаты в метрах. 1 метр будет 15 пикселей
                                   # на рисунке
        p1 = [0, 0]
        p2 = [0, 0]
        p3 = [0, 0]
        p4 = [0, 0]
        for figure in self.all_figures:
            p1[0] = int(512.0 - figure[0][1] * scale) # Вычисляем X первой точки препятствия на нашем рисунке
            p1[1] = int(512.0 - figure[0][0] * scale) # Вычисляем Y первой точки препятствия на нашем рисунке
            p4[0] = int(512.0 - figure[1][1] * scale)
            p4[1] = int(512.0 - figure[1][0] * scale)
            p2[0] = int(512.0 - figure[2][1] * scale)
            p2[1] = int(512.0 - figure[2][0] * scale)
            p3[0] = int(512.0 - figure[3][1] * scale)
            p3[1] = int(512.0 - figure[3][0] * scale)
            # Рисуем текст - номер препятствия
            # cv2.putText(self.img, "Fig #" + str(p[0]), (p1[0], p1[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 0))
            # Рисуем рамку препятствия
            cv2.line(self.img, (p1[0], p1[1]), (p2[0], p2[1]), (255,255,255), 1);
            cv2.line(self.img, (p2[0], p2[1]), (p3[0], p3[1]), (255,255,255), 1);
            cv2.line(self.img, (p3[0], p3[1]), (p4[0], p4[1]), (255,255,255), 1);
            cv2.line(self.img, (p4[0], p4[1]), (p1[0], p1[1]), (255,255,255), 1);

        self.img = cv2.flip(cv2.transpose(self.img), 1)

        x_offset=self.posx
        y_offset=self.posy
        photo = cv2.resize(self.ipm_img, (1 if int(self.ipm_img.shape[0] * self.scale) == 0 else int(self.ipm_img.shape[0] * self.scale),
                                            1 if int(self.ipm_img.shape[1] * self.scale) == 0 else int(self.ipm_img.shape[1] * self.scale)))
        self.img_full[y_offset:y_offset+photo.shape[0], x_offset:x_offset+photo.shape[1]] = photo
        
        # b =  np.copy(self.img_full)
        # place = b[:800, :800]
        # a = self.img[..., :3].repeat(3, axis=2).astype('uint16')
        # place[...] = (place.astype())
        cv2.imshow('Node BEV Builder', self.img)  # Отображаем окно
        cv2.waitKey(1) # Нужно для работы окна



    def calc_points(self):
        self.pts_src[:, 1] = self.pts_src[:, 1] - self.__horizont_line_height
        self.pts_dst[:, 1] = self.pts_dst[:, 1] - self.__horizont_line_height
        
    def load_ipm_config(self):
        config = ConfigLoader("ipm_config").data
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
        image = data.data
        image = np.frombuffer(image, dtype=np.uint8).reshape((data.height, data.width, 4))
        image = PImage.fromarray(image)
        # image.resize((1280, 720))

        self.__img_height = int(image.size[1] * (self.__img_width / image.size[0]))
        image = image.resize((self.__img_width, self.__img_height))
        image = np.array(image)
        # self._logger.info('Image old shape: ' + str(image.shape) )

        ipm_img = self.get_ipm(image)
        ipm_img = cv2.resize(ipm_img, (800, 800))

        # self._logger.info('Image new shape: ' + str(ipm_img.shape))
        # analyze_image = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_RGBA2RGB))
        self.ipm_img = cv2.cvtColor(ipm_img, cv2.COLOR_RGBA2RGB)

        


        
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