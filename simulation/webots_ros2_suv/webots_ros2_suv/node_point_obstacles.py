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
from .lib.config_loader import ConfigLoader
from .lib.coords_transformer import CoordsTransformer
import os

def calc_geo_pos(lat, lon, angle, geosensor_x, geosensor_y, geosensor_y_back, newpoint_x, newpoint_y, lidar_reversed):
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

    # Вычисляем азимут требуемой точки
    if lidar_reversed:
        corr_x = newpoint_x + geosensor_y_back; 
        corr_y = newpoint_y - geosensor_x;
        azimuth = 180 - (math.degrees(math.atan(corr_y / corr_x)) - angle)
        #azimuth = math.degrees(math.atan(corr_x / corr_y)) + angle
    else:
        corr_x = newpoint_x - geosensor_x; 
        corr_y = newpoint_y - geosensor_y;
        
        azimuth = math.degrees(math.atan(corr_x / corr_y)) + angle
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
            self.__world_model = CoordsTransformer()

            self.declare_parameter('start_position')
            self.start_position =  self.get_parameter('start_position').get_parameter_value().integer_value
            self._logger.info(f'Start pos {self.start_position}')


            qos = qos_profile_sensor_data
            qos.reliability = QoSReliabilityPolicy.RELIABLE
            param = ParamLoader()
            # Создаём подписчика, который принимает JSON в виде строки и при приёме данных вызывается функция __on_obstacles_message
            self.create_subscription(String, 'obstacles', self.__on_obstacles_message__old, qos)
            self.__obstdict_pubulisher = self.create_publisher(String, 'obstacles_json', qos)
            # Подписка на GPS данные
            self.create_subscription(Odometry, param.get_param("odom_topicname"), self.__on_gps_message, qos)

            # self.create_subscription(sensor_msgs.msg.Image, param.get_param("front_image_topicname"), self.__on_image_message, qos)
            
            # загружаем локальные конфигурацтлнные файлы
            self.lidardata = ConfigLoader("lidardata").data
            self._logger.info(f"CONFIG_DIRECTORY: " + os.environ.get("CONFIG_DIRECTORY"))

            
            # with open("webots_ros2_suv/config/lidardata.yaml", "r") as file:
                # self.lidardata = yaml.safe_load(file)
            self.MAP_SCALE = self.lidardata['visual_scale']
            self.GPS_SHIFT_X = self.lidardata['gps_shift_x']
            self.GPS_SHIFT_Y = self.lidardata['gps_shift_y']
            self.GPS_SHIFT_Y_BACK = self.lidardata['gps_shift_y_back']
            self.LIDAR_REVERSED = self.lidardata['lidar_reversed']
            self.geocoords = []
            self.rear_figures = []
            self.front_figures = []
            self.obst_dict = []
            self.priority = (12, (52.00153699144721,55.81949941813946), (52.00017778202891,55.81951509229839), 2.0)
            self.lanes = [
                
                (4, (52.000262858346105,55.81925357691944), (52.001535231247544,55.819244692102075), 2.0),
                (6, (52.000265372917056,55.819171350449324), (52.00162634253502,55.819161711260676), 2.0),
                (7, (52.000986719504,55.81941744312644),(52.000986970961094,55.81932163797319), 2.0),
                
                (10, (52.001620307564735,55.81922432407737), (52.00163631699979,55.81968960352242), 2.0),
                #(5, (52.00026554055512,55.81920571625233), (52.001631036400795,55.81919758580625), 2.0),
                (11, (52.001538164913654,55.81953763961792), (52.000093292444944,55.819550547748804), 2.0),
                (12, (52.00153699144721,55.81949941813946), (52.00017778202891,55.81951509229839), 2.0),
                (8, (52.00104648247361,55.819320464506745), (52.00104924850166,55.81940554082394), 2.0),
                (3, (52.00026570819318,55.8192917983979), (52.00153447687626,55.81927880644798), 2.0),
                (9, (52.00156440027058,55.819683484733105), (52.001558281481266,55.81923111341894), 2.0),
                (5, (52.000258415937424,55.819214936345816), (52.00163120403886,55.81920521333814), 2.0),
                (1, (52.00017417781055,55.819496316835284), (52.00016336515546,55.81918191164732), 2.0),
                (2, (52.00023570097983,55.819495394825935), (52.000227738171816,55.819173362106085), 2.0),
                (13, (52.000130005180836,55.81953453831375), (52.00012690387666,55.8191829174757), 0.8),
                (14, (52.000158336013556,55.819165064021945), (52.00162877328694,55.81915215589106), 0.8),
                (15, (52.00167269445956,55.81916967406869), (52.00167923234403,55.81962615251541), 0.8)
            ]
            # self.lanes = [
            #     (1, (52.00017417781055,55.819496316835284), (52.00016336515546,55.81918191164732), 2.0),
            #     (2, (52.00023570097983,55.819495394825935), (52.000227738171816,55.819173362106085), 2.0),
            #     (3, (52.00025984086096,55.81929858773947), (52.00153338722885,55.81928760744631), 2.0),
            #     (4, (52.00026176869869,55.81926195882261), (52.00153389014304,55.81925022415817), 2.0),
            #     (5, (52.000258415937424,55.819214936345816), (52.00163120403886,55.81920521333814), 2.0),
            #     (6, (52.000255482271314,55.819178810343146), (52.00162433087826,55.81916824914515), 2.0),
            #     (7, (52.00099811889231,55.81940218806267),(52.00099552050233,55.819310573861), 2.0),
            #     (8, (52.00106048025191,55.81930789165199), (52.001059809699655,55.819395147264004), 2.0),
            #     (9, (52.001574877649546,55.819640066474676), (52.00156431645155,55.819229101762176), 2.0),
            #     (10, (52.001626091077924,55.81922935321927), (52.00163631699979,55.81968960352242), 2.0),
            #     (11, (52.001538164913654,55.81953763961792), (52.000093292444944,55.819550547748804), 2.0),
            #     (12, (52.00153699144721,55.81949941813946), (52.00017778202891,55.81951509229839), 2.0),
            #     (13, (52.000130005180836,55.81953453831375), (52.00012690387666,55.8191829174757), 0.8),
            #     (14, (52.000158336013556,55.819165064021945), (52.00162877328694,55.81915215589106), 0.8),
            #     (15, (52.00167269445956,55.81916967406869), (52.00167923234403,55.81962615251541), 0.8)
            # ]
            self.object_data = {"ObjectData": []}

            # Вот это нужно получать с odom!!!
            self.lat = 53.0
            self.lon = 48.0
            self.ang = 0
            
        except  Exception as err:
            self._logger.error(''.join(traceback.TracebackException.from_exception(err).format()))
        

        
    def add_lane(self, lane_num, pt1, pt2):
        self.lanes.append((lane_num, pt1, pt2))


    def calc_intermediate(self, pt1, pt2, cnt):
        lons = np.linspace(pt1[0], pt2[0], cnt) # В списке широта и долгота идут наоборот
        lats = np.linspace(pt1[1], pt2[1], cnt) # 
        wp_list = []

        for i in range(lats.size):
            wp_list.append((lats[i], lons[i]))

        return wp_list
        

    def is_in_lane(self, lane, pt):
        geod = Geodesic.WGS84
        dlat = geod.Direct(pt[0], pt[1], 0, lane[3])['lat2'] - pt[0]
        dlon = geod.Direct(pt[0], pt[1], 90, lane[3])['lon2'] - pt[1]

        points = self.calc_intermediate(lane[1], lane[2], 1000)
        for p in points:
            if p[0] - dlat < pt[0] and p[0] + dlat > pt[0] and p[1] - dlon < pt[1] and p[1] + dlon > pt[1]:
                return True
        return False


    def get_lane_number(self, pt):
        for l in self.lanes[::-1]:
            if l[0] in (1, 2) and self.is_in_lane(self.priority, pt):
                return 12
            # self._logger.info(str(l))
            if self.is_in_lane(l, pt):
                return l[0]
        return None


    def __on_gps_message(self, data):
        if self.__world_model:
            roll, pitch, yaw = euler_from_quaternion(data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
            yaw = math.degrees(yaw)
            lat, lon, orientation = self.__world_model.get_global_coords(data.pose.pose.position.x, data.pose.pose.position.y, yaw)
            # self._logger.info(f"yaw: [{lat},{lon}]")

            self.lat, self.lon, self.ang = lon, lat, yaw # Широта и долгота перепутаны!
        pass



    def __on_image_message(self, data):
        image = data.data
        self.backimage = np.frombuffer(image, dtype=np.uint8).reshape((data.height, data.width, 4))
        self.backimage = cv2.cvtColor(self.backimage, cv2.COLOR_RGBA2RGB)
        

    def get_obst_dist(self, p1, p2):
        dist = 1000
        for i in range(4):
            for j in range(4):
                rdist = math.sqrt((p1[i][0] - p2[j][0]) **  2 + (p1[i][0] - p2[j][0]) ** 2)
                if rdist < dist:
                    dist = rdist
        return dist
    
    def add_obstacle(self, obst):
        closer = False

        coords = self.get_corner_coords(obst[0], obst[1], obst[2], obst[3], not obst[4])

        lane_obst = self.get_lane_number((coords['D'][0], coords['D'][1]))
        for p in self.obst_dict:
            p_coords = self.get_corner_coords(p[0], p[1], p[2], p[3], not p[4])
            p_lane = self.get_lane_number((p_coords['D'][0], p_coords['D'][1]))
            # self._logger.info(f"add_obst: dist {self.get_obst_dist(p, obst) }, lane {lane_obst}")
            if self.get_obst_dist(p, obst) < 1.5 and p_lane == lane_obst and p[9] == obst[9]:
                closer = True
        if not closer and lane_obst != None:
            # self._logger.info(f"add_obst: lane {lane_obst}")
            self.obst_dict.append(obst)
        
        
    
    


    def get_corner_coords(self, pt1, pt2, pt3, pt4, is_rear):
        # 1, 4, 2, 3 - по часовой стрелке
        geocoords1 = calc_geo_pos(self.lat, self.lon, self.ang, self.GPS_SHIFT_X, self.GPS_SHIFT_Y, self.GPS_SHIFT_Y_BACK, pt1[0],  pt1[1], is_rear)
        geocoords2 = calc_geo_pos(self.lat, self.lon, self.ang, self.GPS_SHIFT_X, self.GPS_SHIFT_Y, self.GPS_SHIFT_Y_BACK, pt2[0],  pt2[1], is_rear)
        geocoords3 = calc_geo_pos(self.lat, self.lon, self.ang, self.GPS_SHIFT_X, self.GPS_SHIFT_Y, self.GPS_SHIFT_Y_BACK, pt3[0],  pt3[1], is_rear)
        geocoords4 = calc_geo_pos(self.lat, self.lon, self.ang, self.GPS_SHIFT_X, self.GPS_SHIFT_Y, self.GPS_SHIFT_Y_BACK, pt4[0],  pt4[1], is_rear)
        geocoordsD = calc_geo_pos(self.lat, self.lon, self.ang, self.GPS_SHIFT_X, self.GPS_SHIFT_Y, self.GPS_SHIFT_Y_BACK, (pt1[0] + pt4[0]) / 2,  (pt1[1] + pt4[1]) / 2, is_rear)

        max = pt1[0] ** 2 + pt1[1] ** 2
        first = 1;
        if  pt2[0] ** 2 + pt2[1] ** 2 > max:
            max = pt2[0] ** 2 + pt2[1] ** 2
            first = 2;
        if  pt3[0] ** 2 + pt3[1] ** 2 > max:
            max = pt3[0] ** 2 + pt3[1] ** 2
            first = 3;
        if  pt4[0] ** 2 + pt4[1] ** 2 > max:
            max = pt4[0] ** 2 + pt4[1] ** 2
            first = 4;
        if first == 1:
            return {"C": [geocoords1['lat2'], geocoords1['lon2']], 
                    "I": [geocoords4['lat2'], geocoords4['lon2']], 
                    "J": [geocoords2['lat2'], geocoords2['lon2']], 
                    "K": [geocoords3['lat2'], geocoords3['lon2']], 
                    "D": [geocoordsD['lat2'], geocoordsD['lon2']]}
        if first == 2:
            return {"C": [geocoords2['lat2'], geocoords2['lon2']], 
                    "I": [geocoords3['lat2'], geocoords3['lon2']], 
                    "J": [geocoords1['lat2'], geocoords1['lon2']], 
                    "K": [geocoords4['lat2'], geocoords4['lon2']], 
                    "D": [geocoordsD['lat2'], geocoordsD['lon2']]}
        if first == 3:
            return {"C": [geocoords3['lat2'], geocoords3['lon2']], 
                    "I": [geocoords1['lat2'], geocoords1['lon2']], 
                    "J": [geocoords4['lat2'], geocoords4['lon2']], 
                    "K": [geocoords2['lat2'], geocoords2['lon2']], 
                    "D": [geocoordsD['lat2'], geocoordsD['lon2']]}
        return {"C": [geocoords4['lat2'], geocoords4['lon2']], 
                "I": [geocoords2['lat2'], geocoords2['lon2']], 
                "J": [geocoords3['lat2'], geocoords3['lon2']], 
                "K": [geocoords1['lat2'], geocoords1['lon2']], 
                "D": [geocoordsD['lat2'], geocoordsD['lon2']]}

    def count_angle(self, p1, p2, p3):
        v1 = (p2[0] - p1[0], p2[1] - p1[1])
        v2 = (p3[0] - p2[0], p3[1] - p2[1])
        angle = math.atan2(v2[1], v2[0]) - math.atan2(v1[1], v1[0])
        angle = math.degrees(angle)%180
        # если нужен острый угол
        # return min(180 - angle, angle)
        return angle


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
        for p in obst_list:
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

                # Определение класса препятствия
                if abs(p[9] - p[8]) > 1 and abs(p[11] - p[10]) > 1:
                    pclass = 'C'
                else:
                    pclass = 'P'


                if 'obstacles' in obstacles_dict:
                    self.front_figures.append([p[4], p[5], p[6], p[7], True, p[0], p[1], p[2], p[3], pclass])
                else:
                    self.rear_figures.append([p[4], p[5], p[6], p[7], False, p[0], p[1], p[2], p[3], pclass])

        

        self.all_figures = self.front_figures + self.rear_figures

        # Добавляем препятствия в словарь
        for fig in self.all_figures:
            self.add_obstacle(fig)


        # Построение элемента словаря для публикации
        self.object_data['ObjectData'] = []
        for figure in self.obst_dict:
            object_data_item = {"BC": figure[6]} # Дистанция до препятствия

            # Считаем угол alpha - между машиной и центром препятствия
            # (p[4][1] + p[7][1]) / 2,  (p[4][0] + p[7][0]) / 2
            
            pt1 = (0, 1)
            pt2 = (0, 0)
            if figure[4]:
                pt3 = ((figure[0][0] + figure[3][0]) / 2,  (figure[0][1] + figure[3][1]) / 2)
            else:
                pt3 = ((figure[0][1] + figure[3][1]) / -2,  (figure[0][0] + figure[3][0]) / 2)

            object_data_item["alpha"] = self.count_angle(pt1, pt2, pt3)
            p_coords = self.get_corner_coords(figure[0], figure[1], figure[2], figure[3], not figure[4])

            object_data_item["L"] = self.get_lane_number((p_coords['D'][0], p_coords['D'][1])) # Здесь нужно выдавать номер полосы, в которой находится препятствие
            object_data_item["ObjectType"] = "Car" if figure[9] == "C" else "Person"
            object_data_item["Coordinates"] = p_coords

        # UNCOMMINT
            
            self.object_data["ObjectData"].append(object_data_item)
        json_data = String()
        json_data.data = json.dumps(self.object_data)
        self.__obstdict_pubulisher.publish(json_data)
        
        
        
        # self._logger.info("JSON: " + json.dumps(self.object_data))








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

            
            for figure in self.obst_dict:
                if not figure[4]:
                    p1[0] = int(512.0 - figure[0][1] * scale) # Вычисляем X первой точки препятствия на нашем рисунке
                    p1[1] = int(512.0 - figure[0][0] * scale) # Вычисляем Y первой точки препятствия на нашем рисунке
                    p4[0] = int(512.0 - figure[1][1] * scale)
                    p4[1] = int(512.0 - figure[1][0] * scale)
                    p2[0] = int(512.0 - figure[2][1] * scale)
                    p2[1] = int(512.0 - figure[2][0] * scale)
                    p3[0] = int(512.0 - figure[3][1] * scale)
                    p3[1] = int(512.0 - figure[3][0] * scale)
                else:
                    p1[1] = int(512.0 - figure[0][1] * scale) # Вычисляем Y первой точки препятствия на нашем рисунке
                    p1[0] = int(512.0 + figure[0][0] * scale) # Вычисляем X первой точки препятствия на нашем рисунке
                    p4[1] = int(512.0 - figure[1][1] * scale)
                    p4[0] = int(512.0 + figure[1][0] * scale)
                    p2[1] = int(512.0 - figure[2][1] * scale)
                    p2[0] = int(512.0 + figure[2][0] * scale)
                    p3[1] = int(512.0 - figure[3][1] * scale)
                    p3[0] = int(512.0 + figure[3][0] * scale)
                p_coords = self.get_corner_coords(figure[0], figure[1], figure[2], figure[3], not figure[4])

                lanenum = self.get_lane_number((p_coords['D'][0], p_coords['D'][1])) # Здесь нужно выдавать номер полосы, в которой находится препятствие

                # Рисуем текст - номер препятствия
                cv2.putText(self.img, f"[{figure[9]}][{lanenum}], d={figure[6]:.3}", (p1[0], p1[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0));
                # Рисуем рамку препятствия
                cv2.line(self.img, (p1[0], p1[1]), (p2[0], p2[1]), (255,255,255), 1);
                cv2.line(self.img, (p2[0], p2[1]), (p3[0], p3[1]), (255,255,255), 1);
                cv2.line(self.img, (p3[0], p3[1]), (p4[0], p4[1]), (255,255,255), 1);
                cv2.line(self.img, (p4[0], p4[1]), (p1[0], p1[1]), (255,255,255), 1);
                
                cv2.circle(self.img, (p3[0], p3[1]), 3, (255, 255, 0), 1)
            car_lane = self.get_lane_number((self.lat, self.lon))
            cv2.putText(self.img, f"Lane: {car_lane}, lat: {self.lat}, lon: {self.lon}", (10, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0));    
            cv2.imshow('NPO', self.img)  # Отображаем окно
            cv2.waitKey(1) # Нужно для работы окна




    # Функция вызывается, когда прилетает JSON от pcl_map_node
    def __on_obstacles_message__old(self, data):
        
        
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
        for p in obst_list:
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

                # Определение класса препятствия
                if abs(p[9] - p[8]) > 1 and abs(p[11] - p[10]) > 1:
                    pclass = 'C'
                else:
                    pclass = 'P'


                if 'obstacles' in obstacles_dict:
                    self.front_figures.append([p[4], p[5], p[6], p[7], True, p[0], p[1], p[2], p[3], pclass])
                else:
                    self.rear_figures.append([p[4], p[5], p[6], p[7], False, p[0], p[1], p[2], p[3], pclass])

        self.all_figures = self.front_figures + self.rear_figures


        # Построение элемента словаря для публикации
        for figure in self.all_figures:
            object_data_item = {"BC": figure[6]} # Дистанция до препятствия

            # Считаем угол alpha - между машиной и центром препятствия
            # (p[4][1] + p[7][1]) / 2,  (p[4][0] + p[7][0]) / 2
            
            pt1 = (0, 1)
            pt2 = (0, 0)
            if figure[4]:
                pt3 = ((figure[0][0] + figure[3][0]) / 2,  (figure[0][1] + figure[3][1]) / 2)
            else:
                pt3 = ((figure[0][1] + figure[3][1]) / -2,  (figure[0][0] + figure[3][0]) / 2)

            object_data_item["alpha"] = self.count_angle(pt1, pt2, pt3)
            p_coords = self.get_corner_coords(figure[0], figure[1], figure[2], figure[3], not figure[4])

            object_data_item["L"] = self.get_lane_number((p_coords['D'][0], p_coords['D'][1])) # Здесь нужно выдавать номер полосы, в которой находится препятствие
            object_data_item["ObjectType"] = "Car" if figure[9] == "C" else "Person"
            object_data_item["Coordinates"] = p_coords

            self.object_data["ObjectData"].append(object_data_item)
            
        
        # self._logger.info("JSON: " + json.dumps(self.object_data))








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
                if not figure[4]:
                    p1[0] = int(512.0 - figure[0][1] * scale) # Вычисляем X первой точки препятствия на нашем рисунке
                    p1[1] = int(512.0 - figure[0][0] * scale) # Вычисляем Y первой точки препятствия на нашем рисунке
                    p4[0] = int(512.0 - figure[1][1] * scale)
                    p4[1] = int(512.0 - figure[1][0] * scale)
                    p2[0] = int(512.0 - figure[2][1] * scale)
                    p2[1] = int(512.0 - figure[2][0] * scale)
                    p3[0] = int(512.0 - figure[3][1] * scale)
                    p3[1] = int(512.0 - figure[3][0] * scale)
                else:
                    p1[1] = int(512.0 - figure[0][1] * scale) # Вычисляем Y первой точки препятствия на нашем рисунке
                    p1[0] = int(512.0 + figure[0][0] * scale) # Вычисляем X первой точки препятствия на нашем рисунке
                    p4[1] = int(512.0 - figure[1][1] * scale)
                    p4[0] = int(512.0 + figure[1][0] * scale)
                    p2[1] = int(512.0 - figure[2][1] * scale)
                    p2[0] = int(512.0 + figure[2][0] * scale)
                    p3[1] = int(512.0 - figure[3][1] * scale)
                    p3[0] = int(512.0 + figure[3][0] * scale)
                p_coords = self.get_corner_coords(figure[0], figure[1], figure[2], figure[3], not figure[4])

                lanenum = self.get_lane_number((p_coords['D'][0], p_coords['D'][1])) # Здесь нужно выдавать номер полосы, в которой находится препятствие

                # Рисуем текст - номер препятствия
                # cv2.putText(self.img, f"[{figure[9]}][{lanenum}]Fig #{figure[5]}, d={figure[6]}, hmin={figure[7]}, hmax={figure[8]}", (p1[0], p1[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0));
                cv2.putText(self.img, f"[{figure[9]}][{lanenum}], d={figure[6]:.3}", (p1[0], p1[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0));

                # Рисуем рамку препятствия
                cv2.line(self.img, (p1[0], p1[1]), (p2[0], p2[1]), (255,255,255), 1);
                cv2.line(self.img, (p2[0], p2[1]), (p3[0], p3[1]), (255,255,255), 1);
                cv2.line(self.img, (p3[0], p3[1]), (p4[0], p4[1]), (255,255,255), 1);
                cv2.line(self.img, (p4[0], p4[1]), (p1[0], p1[1]), (255,255,255), 1);
                
                cv2.circle(self.img, (p3[0], p3[1]), 3, (255, 255, 0), 1)
            car_lane = self.get_lane_number((self.lat, self.lon))
            cv2.putText(self.img, f"Lane: {car_lane}, lat: {self.lat}, lon: {self.lon}", (10, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0));    
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