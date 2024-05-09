import rclpy
import os
import cv2
import pathlib
import yaml
import numpy as np
import math
from ament_index_python.packages import get_package_share_directory
from .car_model import CarModel
from .coords_transformer import CoordsTransformer
from webots_ros2_suv.lib.lane_line_model_utils import get_label_names, draw_lines, draw_segmentation, LaneLine, LaneMask


class WorldModel(object):
    '''
    Класс, моделирующий параметры внешней среды в привязке к глобальной карте.
    '''
    def __init__(self):
        self.__car_model = CarModel()
        self.coords_transformer = CoordsTransformer()
        
        self.path = None            # спланированный путь
        self.gps_path = None        # спланированный путь в глобальных координатах
        self.rgb_image = None       # цветное изображение с камеры
        self.range_image = None     # изображение с камеры глубины
        self.point_cloud = None     # облако точек от лидара
        self.seg_image = None       # сегментированное изображение во фронтальной проекции
        self.seg_colorized = None   # раскрашенное сегментированное изображение во фронтальной проекции
        self.seg_composited = None  # раскрашенное сегментированное изображение во фронтальной проекции
        self.lane_contours = None   # контуры линий дорожной разметки на изображении во фронтальной проекции
        self.lane_lines = None      # линии дорожной разметки на изображении во фронтальной проекции
        self.lane_contours_bev = None # контуры линий дорожной разметки на изображении в BEV проекции
        self.lane_lines_bev = None  # линии дорожной разметки на изображении во BEV проекции
        self.img_front_objects = None # изображение с камеры с детектированными объектами
        self.img_front_objects_lines = None # изображение с камеры с детектированными объектами + линии дорожной разметки
        self.objects = None         # объекты во фронтальной проекции   
        self.ipm_image = None       # BEV сегментированное изображение 
        self.ipm_colorized = None   # раскрашенное BEV сегментированное изображение
        self.ipm_colorized_lines = None   # раскрашенное BEV сегментированное изображение + линии дорожной разметки
        self.map_builder = None     # класс, хранящий BEV матрицу
        self.pov_point = None       # Точка в BEV, соответствующая арсположению авто
        self.goal_point = None      # Точка в BEV, соответствующая цели
        self.global_map = None      # текущие загруженные координаты точек глобальной карты
        self.cur_path_segment = 0   # Текущий сегмент пути, заданный в редакторе карт
        self.cur_turn_polygon = None# Текущий полигон для разворота
        self.command_message = None # Сообщение типа AckermanDrive для движения автомобиля


    def load_map(self, mapyaml):
        self.global_map = []
        for f in mapyaml['features']:
            self.global_map.append({
                'name': f['properties']['id'].replace('_point', ''),
                'type': f['geometry']['type'],
                'coordinates': f['geometry']['coordinates']
            })

    def get_current_position(self):
        return self.__car_model.get_position()

    def draw_scene(self):
        colorized = self.ipm_colorized
        prev_point = None
        if self.path:
            for n in self.path:
                if prev_point:
                    cv2.line(colorized, prev_point, n, (0, 255, 255), 2)
                prev_point = n
        cv2.circle(colorized, self.pov_point, 9, (0, 255, 0), 5)
        cv2.circle(colorized, self.goal_point, 9, (255, 0, 0), 5)
        points = [e['coordinates'] for e in self.global_map if e['name'] == 'moving'][self.cur_path_segment]

        font = cv2.FONT_HERSHEY_SIMPLEX 
        fontScale = 1
        color = (255, 255, 0) 
        thickness = 2
        for i, p in enumerate(points):
            x, y = self.coords_transformer.get_relative_coordinates(p[0], p[1], self.get_current_position(), self.pov_point)
            cv2.circle(colorized, (x, y), 8, (0, 0, 255), 2)
            image = cv2.putText(colorized, f'{i}', (x + 20, y), font,  fontScale, color, thickness, cv2.LINE_AA)

        colorized = cv2.resize(colorized, (500, 500), cv2.INTER_AREA)


        # cv2.imshow("colorized seg", colorized)
        # cv2.imshow("yolo drawing", self.img_front_objects)


        #cv2.imshow("composited image", np.asarray(colorize(world_model.ipm_seg)))
        #img_tracks = draw_absolute_tracks(self.__track_history_bev, 500, 500, self._logger)
        #cv2.imshow("yolo drawing", img_tracks)


        # if cv2.waitKey(10) & 0xFF == ord('q'):
        #     return

    def get_speed(self):
        return self.__car_model.get_speed()
    
    def set_speed(self, speed):
        self.__car_model.set_speed(speed)

    def update_car_pos(self, lat, lon, orientation):        
        self.__car_model.update(lat=lat, lon=lon, orientation=orientation)
