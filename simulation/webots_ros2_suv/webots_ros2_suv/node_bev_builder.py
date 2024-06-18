
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



# Класс для всего узла
class NodeBEVBuilder(Node):
    def __init__(self):
        try:

            # Инициализация узла
            super().__init__('node_bev_builder')
            self._logger.info('Node BEV Builder Started')
            param = ParamLoader()

            qos = qos_profile_sensor_data
            qos.reliability = QoSReliabilityPolicy.RELIABLE

            self.__img_height = 720
            self.__img_width = 1280
            
            #self.src_x_list = list()
            #self.src_y_list = list()
            #self.dst_x_list = list()
            #self.dst_y_list = list()
            #for i in range(0, 4):
            #    varx = 0
            #    vary = 0
            #    self.src_x_list.append(varx)
            #    self.src_y_list.append(vary)

            #    varx_dst = 0
            #    vary_dst = 0
            #    self.dst_x_list.append(varx_dst)
            #    self.dst_y_list.append(vary_dst)
            pts_src, pts_dst = self.calc_pts(self.__img_width, self.__img_height)

            self.calc_homography(pts_src, pts_dst)

            # Создаём подписчика, который принимает JSON в виде строки и при приёме данных вызывается функция __on_obstacles_message
            self.create_subscription(String, 'obstacles', self.__on_obstacles_message, qos)
            self.create_subscription(sensor_msgs.msg.Image, param.get_param("front_image_topicname"), self.__on_image_message, qos)
        except  Exception as err:
            self._logger.error(''.join(traceback.TracebackException.from_exception(err).format()))
        
    def calc_homography(self, pts_src, pts_dst):
        # Метод для вычисления матрицы гомографии на основе набора исходных точек (pts_src) и соответствующих точек назначения (pts_dst).
        # Использует функцию cv2.findHomography для выполнения вычислений.
        # Возвращает результат в виде матрицы гомографии (self.__h).
        self.__h, status = cv2.findHomography(pts_src, pts_dst, cv2.RANSAC)
        return self.__h

    def calc_pts(self, width, height):
# For 800 px
        pts_src = np.array([[int(width / 2.5) - 45, int(height / 2.5) + 15],
                            [int(width / 4) - 112, int(3 * height / 4) + 85],
                            [int(3 * width / 4) - 50, int(height / 2.5) + 15],
                            [int(3 * width / 4) + 189, int(3 * height / 4) + 85]])

        pts_dst = np.array([[int(width / 2.5), int(height / 2.5) + 15],
                            [int(width / 4) + 115, int(3 * height / 4) + 85],
                            [int(3 * width / 4) - 10, int(height / 2.5) + 15],
                            [int(3 * width / 4) + 5, int(3 * height / 4) + 85]])
# # For 400 px
#         pts_src = np.array([[int(width / 2) + 100, int(height / 2) + 80],
#                             [int(width / 4) + 140, int(height / 4) + 360],
#                             [int(width / 4) + 80, int(height / 2) + 80],
#                             [int(width / 4) + 200, int(height / 4) + 200]])

#         pts_dst = np.array([[int(width / 2), int(height / 2) ],
#                             [int(width / 4), int(height / 4)],
#                             [int(width / 4), int(height / 2)],
#                             [int(width / 4), int(height / 4)]])
        
        # or i in range(4):
        #    self.src_x_list[i] = int(pts_src[i][0])
        #    self.src_y_list[i] = int(pts_src[i][1])

        #    self.dst_x_list[i] = int(pts_dst[i][0])
        #    self.dst_y_list[i] = int(pts_dst[i][1])
        return pts_src, pts_dst
    

    def get_ipm(self, im_src, dst_size=(1200, 800), horizont=0, is_mono=False, need_cut=True):
        # Метод для выполнения обратного преобразования перспективы (Inverse Perspective Mapping) на исходном изображении im_src.
        # im_src - исходное изображение, на которое будет применено обратное преобразование перспективы.
        # dst_size - размер целевого изображения после преобразования (по умолчанию (1200, 800)).
        # horizont - высота горизонтальной линии, используемой для обрезки изображения (по умолчанию 0).
        # is_mono - флаг, указывающий, является ли исходное изображение монохромным (по умолчанию False).
        # need_cut - флаг, указывающий, нужно ли обрезать изображение для удаления нулевых строк и столбцов в результате (по умолчанию True).
        # Возвращает преобразованное изображение после применения обратного преобразования перспективы.
        im_src = im_src[horizont:, :]  # Обрезаем изображение по горизонтали, если задана высота горизонтали
        im_dst = cv2.warpPerspective(im_src, self.__h, dst_size)  # Применяем матрицу гомографии
        if not need_cut:
            return im_dst
        if not is_mono:
            rows = np.sum(im_dst, axis=(1, 2))
            cols = np.sum(im_dst, axis=(0, 2))
        else:
            rows = np.sum(im_dst, axis=(1))
            cols = np.sum(im_dst, axis=(0))

        nonzero_rows = len(rows[np.nonzero(rows)])  # Находим ненулевые строки
        nonzero_cols = len(cols[np.nonzero(cols)])  # Находим ненулевые столбцы
        im_dst = im_dst[:nonzero_rows, :nonzero_cols]  # Обрезаем изображение до ненулевых строк и столбцов
        im_dst = cv2.resize(im_dst, (im_dst.shape[1], im_dst.shape[1]), interpolation=cv2.INTER_AREA)  # Изменяем размер
        return im_dst

    def __on_image_message(self, data):
        image = data.data
        image = np.frombuffer(image, dtype=np.uint8).reshape((data.height, data.width, 4))
        image = PImage.fromarray(image)
        # image.resize((1280, 720))

        self.__img_height = int(image.size[1] * (self.__img_width / image.size[0]))
        image = image.resize((self.__img_width, self.__img_height))
        image = np.array(image)
        self._logger.info('Image old shape: ' + str(image.shape) )

        image = self.get_ipm(image)
        self._logger.info('Image new shape: ' + str(image.shape))
        # analyze_image = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_RGBA2RGB))
        cv2.imshow('Node BEV Builder', image)  # Отображаем окно
        cv2.waitKey(1) # Нужно для работы окна


        
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