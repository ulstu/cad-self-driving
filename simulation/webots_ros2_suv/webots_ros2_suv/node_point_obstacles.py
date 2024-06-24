
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
from geographiclib.geodesic import Geodesic
from .lib.param_loader import ParamLoader
from .lib.config_loader import GlobalConfigLoader


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
            param = ParamLoader()
            # Создаём подписчика, который принимает JSON в виде строки и при приёме данных вызывается функция __on_obstacles_message
            self.create_subscription(String, 'obstacles', self.__on_obstacles_message, qos)
            self.create_subscription(sensor_msgs.msg.Image, param.get_param("front_image_topicname"), self.__on_image_message, qos)
            
            self.lidardata = GlobalConfigLoader("lidardata").data

            self.MAP_SCALE = self.lidardata['visual_scale']
            self.GPS_SHIFT_X = self.lidardata['gps_shift_x']
            self.GPS_SHIFT_Y = self.lidardata['gps_shift_y']
            self.geocoords = []
            self.rear_figures = []
            self.front_figures = []
            self.lat = 53.0
            self.lon = 48.0
            self.ang = 0
        except  Exception as err:
            self._logger.error(''.join(traceback.TracebackException.from_exception(err).format()))
        

    def __on_image_message(self, data):
        image = data.data
        self.backimage = np.frombuffer(image, dtype=np.uint8).reshape((data.height, data.width, 4))
        self.backimage = cv2.cvtColor(self.backimage, cv2.COLOR_RGBA2RGB)
        
    



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
        p1 = [0, 0]
        p2 = [0, 0]
        p3 = [0, 0]
        p4 = [0, 0]
        # Обходим все обнаруженные препятствия
        self.geocoords = []
        for p in obst_list:
            # Геокоординаты 4-х точек - углов препятствия
            pgeocoords = calc_geo_pos(self.lat, self.lon, self.ang, self.GPS_SHIFT_X, self.GPS_SHIFT_Y, p[4][1],  p[4][0])
            self.geocoords.append([pgeocoords['lat2'], pgeocoords['lon2']])
            pgeocoords = calc_geo_pos(self.lat, self.lon, self.ang, self.GPS_SHIFT_X, self.GPS_SHIFT_Y, p[5][1],  p[4][0])
            self.geocoords.append([pgeocoords['lat2'], pgeocoords['lon2']])
            pgeocoords = calc_geo_pos(self.lat, self.lon, self.ang, self.GPS_SHIFT_X, self.GPS_SHIFT_Y, p[6][1],  p[4][0])
            self.geocoords.append([pgeocoords['lat2'], pgeocoords['lon2']])
            pgeocoords = calc_geo_pos(self.lat, self.lon, self.ang, self.GPS_SHIFT_X, self.GPS_SHIFT_Y, p[7][1],  p[4][0])
            self.geocoords.append([pgeocoords['lat2'], pgeocoords['lon2']])
            # Геокоординаты середины диагонали препятствия
            pgeocoords = calc_geo_pos(self.lat, self.lon, self.ang, self.GPS_SHIFT_X, self.GPS_SHIFT_Y, (p[4][1] + p[7][1]) / 2,  (p[4][0] + p[7][0]) / 2) 
            self.geocoords.append([pgeocoords['lat2'], pgeocoords['lon2']])




            if self.lidardata['visualize']:
                # здесь будут координаты углов препятствий

            
                # p[0] - номер препятствия
                # p[1] - расстояние до ближайшей точки препятствия
                # p[2] - высота самой нижней точки препятствия относительно датчика
                # p[3] - высота самой верхней точки препятствия относительно датчика
                # p[4], p[5], p[6], p[7] - списки из двух чисел - координаты углов препятствия
                # p[8] - xmin
                # p[9] - xmax
                # p[10] - ymin
                # p[11] - ymax
                if 'obstacles' in obstacles_dict:
                    self.front_figures.append([p[4], p[5], p[6], p[7], True, p[0], p[1], p[2], p[3]])
                else:
                    self.rear_figures.append([p[4], p[5], p[6], p[7], False, p[0], p[1], p[2], p[3]])

        self.all_figures = self.front_figures + self.rear_figures
        # self._logger.info(str(self.geocoords))

        if self.lidardata['visualize']:
            

            # Создаём черный фон
            self.img = np.zeros(shape=(1024, 1024, 3), dtype=np.uint8)
            x_offset=y_offset=50
            if hasattr(self, "backimage"):
                self.img[y_offset:y_offset+self.backimage.shape[0], x_offset:x_offset+self.backimage.shape[1]] = self.backimage
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
                cv2.putText(self.img, "Fig #" + str(figure[5]) + ", d = " + str(figure[6]) + ", hmin = " + str(figure[7]) + ", hmax = " + str(figure[8]), (p1[0], p1[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0))
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
