import os
import cv2
import pathlib
import yaml
import numpy as np
import math
from ament_index_python.packages import get_package_share_directory


class CoordsTransformer(object):
    def __init__(self) -> None:
        # корректировка координат, необходима для установления соотвествия между координатами симулятора и OSM карты
        # кортеж значений (x, y, direction)
        self.__coord_corrections = (0, 0, 0, 0)
        self.__load_config()
        self.__EARTH_RADIUS_KM = 6378.137


    def __load_config(self):
        package_dir = get_package_share_directory('webots_ros2_suv')
        config_path = os.path.join(package_dir,
                                    pathlib.Path(os.path.join(package_dir, 'config', 'global_coords.yaml')))
        if not os.path.exists(config_path):
            print('Global map coords config file file not found. Use default values')
            return
        with open(config_path) as file:
            config = yaml.full_load(file)
        self.__coord_corrections = (config['lat'], config['lon'], config['orientation'],  config['scale_x'], config['scale_y'], config['bev_orientation'])
        print('Translation coordinates: ', self.__coord_corrections)

    def __get_latitude(self, latitude: float, meters: float) -> float:
        m: float = (1 / ((2 * math.pi / 360) * self.__EARTH_RADIUS_KM)) / 1000
        return latitude + (meters * m)

    def __get_longitude(self, longitude: float, meters: float) -> float:
        latitude: float = 0.0
        m: float = (1 / ((2 * math.pi / 360) * self.__EARTH_RADIUS_KM)) / 1000
        return longitude + (meters * m) / math.cos(latitude * (math.pi / 180))

    def get_global_coords(self, lat, lon, yaw):
        latitude = self.__get_latitude(self.__coord_corrections[0], lat)
        longitude = self.__get_longitude(self.__coord_corrections[1], lon)
        o = self.__coord_corrections[2] - yaw
        return latitude, longitude, o

    def get_relative_coordinates(self, target_lat, target_lon,  pos, pov_point):
        # Разбиваем кортеж на составляющие
        start_lat, start_lon, start_angle, scale_x, scale_y = pos[0], pos[1], pos[2], self.__coord_corrections[3], self.__coord_corrections[4]
        start_angle = start_angle - self.__coord_corrections[5]
        # Пересчитываем разницу в метрах для широты и долготы
        delta_lat_meters = self.__delta_latitude_in_meters(target_lat, start_lat)
        delta_lon_meters = self.__delta_longitude_in_meters(target_lon, start_lon, start_lat)
 
        # # Применяем поворот
        rotated_x, rotated_y = self.__rotate_coordinates(delta_lat_meters, delta_lon_meters, start_angle)

        # # Применяем масштабирование
        scaled_x = rotated_x * scale_x
        scaled_y = rotated_y * scale_y

        res_scaled_x = int(pov_point[0] + scaled_x) if (pov_point[0] + scaled_x) >=0 else 0
        res_scaled_y = int(pov_point[1] - scaled_y) if (pov_point[1] - scaled_y) >=0 else 0

        return (int(res_scaled_x), int(res_scaled_y))

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


    def get_global_coordinates_from_ipm_coords(self, relative_x, relative_y, pos):
        # Разбиваем кортеж на составляющие
        start_lat, start_lon, start_angle, scale_x, scale_y, bev_orientation = pos[0], pos[1], pos[2], self.__coord_corrections[3], self.__coord_corrections[4], self.__coord_corrections[5]
        #start_lat, start_lon, start_angle, scale_x, scale_y, bev_orientation = self.__coord_corrections

        # Применяем обратное масштабирование
        unscaled_x = relative_x / scale_x
        unscaled_y = relative_y / scale_y

        # Применяем обратный поворот
        start_angle = start_angle - bev_orientation
        angle_rad = math.radians(-start_angle)  # Обратный угол
        rotated_x = unscaled_x * math.cos(angle_rad) + unscaled_y * math.sin(angle_rad)
        rotated_y = -unscaled_x * math.sin(angle_rad) + unscaled_y * math.cos(angle_rad)

        # Преобразуем относительные координаты в глобальные
        latitude = self.__get_latitude_back(start_lat, rotated_y)
        longitude = self.__get_longitude_back(start_lon, rotated_x, start_lat)

        return latitude, longitude

    def __get_latitude_back(self, start_lat, delta_meters):
        m = (1 / ((2 * math.pi / 360) * self.__EARTH_RADIUS_KM)) / 1000
        return start_lat - (delta_meters * m)

    def __get_longitude_back(self, start_lon, delta_meters, start_lat):
        m = (1 / ((2 * math.pi / 360) * self.__EARTH_RADIUS_KM)) / 1000
        cos_lat = math.cos(start_lat * (math.pi / 180))
        return start_lon - (delta_meters * m) / cos_lat        
    
