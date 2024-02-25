import rclpy
import os
import pathlib
import yaml
import numpy as np
import math
from ament_index_python.packages import get_package_share_directory
from .car_model import CarModel

class WorldModel(object):
    '''
    Класс, моделирующий параметры внешней среды в привязке к глобальной карте.
    '''
    def __init__(self):
        self.__car_model = CarModel()
        # корректировка координат, необходима для установления соотвествия между координатами симулятора и OSM карты
        # кортеж значений (x, y, direction)
        self.__coord_corrections = (0, 0, 0, 0)
        self.__load_config()
        
        self.path = None            # спланированный путь
        self.rgb_image = None       # цветное изображение с камеры
        self.range_image = None     # изображение с камеры глубины
        self.point_cloud = None     # облако точек от лидара
        self.seg_image = None       # сегментированное изображение во фронтальной проекции
        self.seg_colorized = None   # раскрашенное сегментированное изображение во фронтальной проекции
        self.seg_composited = None   # раскрашенное сегментированное изображение во фронтальной проекции
        self.objects = None         # объекты во фронтальной проекции   
        self.ipm_image = None       # BEV сегментированное изображение 
        self.ipm_colorized = None   # раскрашенное BEV сегментированное изображение
        self.pov_point = None       # Точка в BEV, соответствующая арсположению авто
        self.goal_point = None      # Точка в BEV, соответствующая цели
        self.global_map = None      # текущие загруженные координаты точек глобальной карты

        self.__EARTH_RADIUS_KM = 6378.137

    def __get_latitude(self, latitude: float, meters: float) -> float:
        m: float = (1 / ((2 * math.pi / 360) * self.__EARTH_RADIUS_KM)) / 1000
        return latitude + (meters * m)

    def __get_longitude(self, longitude: float, meters: float) -> float:
        latitude: float = 0.0
        m: float = (1 / ((2 * math.pi / 360) * self.__EARTH_RADIUS_KM)) / 1000
        return longitude + (meters * m) / math.cos(latitude * (math.pi / 180))

    def __load_config(self):
        package_dir = get_package_share_directory('webots_ros2_suv')
        config_path = os.path.join(package_dir,
                                    pathlib.Path(os.path.join(package_dir, 'config', 'global_coords.yaml')))
        if not os.path.exists(config_path):
            print('Global map coords config file file not found. Use default values')
            return
        with open(config_path) as file:
            config = yaml.full_load(file)
        self.__coord_corrections = (config['lat'], config['lon'], config['orientation'],  config['scale_x'], config['scale_y'])
        print('Translation coordinates: ', self.__coord_corrections)

    def load_map(self, mapyaml):
        self.global_map = []
        for f in mapyaml['features']:
            self.global_map.append({
                'name': f['properties']['id'].replace('_point', ''),
                'type': f['geometry']['type'],
                'coordinates': f['geometry']['coordinates']
            })

    def get_global_coords(self, lat, lon, yaw):
        latitude = self.__get_latitude(self.__coord_corrections[0], lat)
        longitude = self.__get_longitude(self.__coord_corrections[1], lon)
        o = self.__coord_corrections[2] - yaw
        self.__car_model.update(lat=latitude, lon=longitude, orientation=o)
        return latitude, longitude, o

    def get_current_position(self):
        return self.__car_model.get_position()
    
    def get_relative_coordinates(self, target_lat, target_lon):
        # Разбиваем кортеж на составляющие
        pos = self.get_current_position()
        start_lat, start_lon, start_angle, scale_x, scale_y = pos[0], pos[1], pos[2], self.__coord_corrections[3], self.__coord_corrections[4]
        #start_angle = math.pi - start_angle

        # Пересчитываем разницу в метрах для широты и долготы
        delta_lat_meters = self.__delta_latitude_in_meters(target_lat, start_lat)
        delta_lon_meters = self.__delta_longitude_in_meters(target_lon, start_lon, start_lat)
 
        # # Применяем поворот
        rotated_x, rotated_y = self.__rotate_coordinates(delta_lat_meters, delta_lon_meters, start_angle)

        # # Применяем масштабирование
        scaled_x = rotated_x * scale_x
        scaled_y = rotated_y * scale_y

        scaled_x = int(self.pov_point[0] + scaled_x) if self.pov_point[0] + scaled_x >=0 else 0
        scaled_y = int(self.pov_point[1] - scaled_y) if self.pov_point[1] - scaled_y >=0 else 0

        return (int(scaled_x), int(scaled_y))

    def __delta_latitude_in_meters(self, target_lat, start_lat):
        """
        Вычисляет разницу в метрах между двумя широтами.
        """
        delta_degrees = target_lat - start_lat
        delta_meters = delta_degrees * (2 * math.pi * self.__EARTH_RADIUS_KM * 1000) / 360
        return delta_meters

    def __delta_longitude_in_meters(self, target_lon, start_lon, start_lat):
        """
        Вычисляет разницу в метрах между двумя долготами, учитывая широту.
        """
        delta_degrees = target_lon - start_lon
        # Рассчитываем длину дуги одного градуса долготы в метрах на данной широте
        arc_length_per_degree = math.cos(start_lat * math.pi / 180) * (2 * math.pi * self.__EARTH_RADIUS_KM * 1000) / 360
        delta_meters = delta_degrees * arc_length_per_degree
        return delta_meters

    def __rotate_coordinates(self, x, y, angle):
        # Поворот координат на угол angle
        rotated_x = x * math.cos(angle) - y * math.sin(angle)
        rotated_y = x * math.sin(angle) + y * math.cos(angle)
        return rotated_x, rotated_y  

    def get_coord_corrections(self):
        return self.__coord_corrections  

