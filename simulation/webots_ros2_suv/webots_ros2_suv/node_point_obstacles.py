
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
from PIL import Image
from .lib.timeit import timeit
from .lib.world_model import WorldModel
from .lib.orientation import euler_from_quaternion
from .lib.finite_state_machine import FiniteStateMachine
from .lib.map_server import start_web_server, MapWebServer
import threading
from sensor_msgs.msg import Image, NavSatFix, NavSatStatus, PointCloud2, PointField
from robot_interfaces.srv import PoseService
from robot_interfaces.msg import EgoPose
from geographiclib.geodesic import Geodesic


def calc_geo_pos(lat, lon, angle, geosensor_x, geosensor_y, newpoint_x, newpoint_y):
    """
    Функция вычисления географических координат точки
    На вход подаются:
    lat, lon - широта и долгота GPS-сенсора
    angle - угол установки GPS-сенсора отностельно севера по часовой стрелке В ГРАДУСАХ!!!
    geosensor_x, geosensor_y - позиция GPS-сенсора относительно лидара (точки 0, 0 в системе декартовых координат)
    newpoint_x, newpoint_y - декартовые координаты точки, географические координаты которой нужно посчитать
    На выходе получается словарь, содержащий lat и lon требуемой точки (lat2, lon2)
    
    По координатам: x больше - точка дальше вперёд от нас, y больше - точка правее от нас
    """
    geod = Geodesic.WGS84
    # Считаем координаты точки относительно GPS-сенсора
    corr_x = newpoint_x - geosensor_x; 
    corr_y = newpoint_y - geosensor_y;
    # Вычисляем азимут требуемой точки
    azimuth = math.degrees(math.atan(corr_y / corr_x)) + angle
    # Вычисляем расстоние до требуемой точки
    shift = math.sqrt(corr_x ** 2 + corr_y ** 2)
    # Вычисляем и возвращаем географические координаты точки
    return geod.Direct(lat, lon, azimuth, shift)




# Класс для всего узла
class NodePointObstacles(Node):
    def __init__(self):
        try:

            # Инициализация узла
            super().__init__('node_point_obstacles')
            self._logger.info('Node Point Obstacles Started')


            qos = qos_profile_sensor_data
            qos.reliability = QoSReliabilityPolicy.RELIABLE
    
            # Создаём подписчика, который принимает JSON в виде строки и при приёме данных вызывается функция __on_obstacles_message
            self.create_subscription(String, 'obstacles', self.__on_obstacles_message, qos)
            with open("src/webots_ros2_suv/config/lidardata.yaml", "r") as file:
                self.lidardata = yaml.safe_load(file)
            self.MAP_SCALE = self.lidardata['visual_scale']
            self.geocoords = []
            self.rear_figures = []
            self.front_figures = []
        except  Exception as err:
            self._logger.error(''.join(traceback.TracebackException.from_exception(err).format()))
        

    # Функция вызывается, когда прилетает JSON от pcl_map_node
    def __on_obstacles_message(self, data):
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
            if self.lidardata['visualize']:
                # здесь будут координаты углов препятствий
                p1 = [0, 0]
                p2 = [0, 0]
                p3 = [0, 0]
                p4 = [0, 0]
            
                # p[0] - номер препятствия
                # p[1] - расстояние до ближайшей точки препятствия
                # p[2] - высота самой нижней точки препятствия относительно датчика
                # p[3] - высота самой верхней точки препятствия относительно датчика
                # p[4], p[5], p[6], p[7] - списки из двух чисел - координаты углов препятствия
                if 'obstacles' in obstacles_dict:
                    self.front_figures.append([p[4], p[5], p[6], p[7]])
                else:
                    self.rear_figures.append([p[4], p[5], p[6], p[7]])

        

        if self.lidardata['visualize']:
            self.all_figures = self.front_figures + self.rear_figures

            # Создаём черный фон
            self.img = np.zeros(shape=(1024, 1024, 3), dtype=np.uint8)
            # Вычисляем координаты для отрисовки. В JSON координата X направлена от нас вдаль. Чем X больше, тем точка от нас дальше
            # Поэтому x становится на отображении координатой Y. Чем она больше, тем точка должна быть выше. Поэтому отнимаем её от 800
            # Координата y в JSON - смещение перпендикулярно нашей оси взгляда. Чем эта y бо(льше, тем точка от нас левее. Поэтому в отрисовке
            # она становится координатой X и мы вычитаем её из середины нашего окна. А у середины окна координаты (512, 512)


            scale = self.MAP_SCALE # Домножаем на scale, чтобы картинка не была слишком мелкой, т.к. координаты в метрах. 1 метр будет 15 пикселей
                                   # на рисунке

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
                cv2.putText(self.img, "Fig #" + str(p[0]), (p1[0], p1[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 0))
                # Рисуем рамку препятствия
                cv2.line(self.img, (p1[0], p1[1]), (p2[0], p2[1]), (255,255,255), 1);
                cv2.line(self.img, (p2[0], p2[1]), (p3[0], p3[1]), (255,255,255), 1);
                cv2.line(self.img, (p3[0], p3[1]), (p4[0], p4[1]), (255,255,255), 1);
                cv2.line(self.img, (p4[0], p4[1]), (p1[0], p1[1]), (255,255,255), 1);
        
            cv2.imshow('NPO', self.img)  # Отображаем окно
            cv2.waitKey(1) # Нужно для работы окна


        
# Запуск и работа всей ноды
def main(args=None):
    try:
        rclpy.init(args=args) 
        point_controller = NodePointObstacles()
        rclpy.spin(point_controller)
        rclpy.shutdown()
    except KeyboardInterrupt:
        print('server stopped cleanly')
    except  Exception as err:
        print(''.join(traceback.TracebackException.from_exception(err).format()))
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()