import math
import os
import pathlib
from geopy.distance import geodesic, distance

import cv2
import numpy as np
import yaml
import math
from ament_index_python.packages import get_package_share_directory
from webots_ros2_suv.lib.config_loader import ConfigLoader


class CoordsTransformer(object):
    def __init__(self) -> None:
        # корректировка координат, необходима для установления соотвествия между координатами симулятора и OSM карты
        # кортеж значений (x, y, direction)
        self.__coord_corrections = (0, 0, 0, 0)
        self.__load_config()
        self.__EARTH_RADIUS_KM = 6378.137


    def __load_config(self):
        config = ConfigLoader("global_coords").data
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
        if self.__coord_corrections[0] > 0.0 and self.__coord_corrections[1] > 0.0:
            latitude = self.__get_latitude(self.__coord_corrections[0], lat)
            longitude = self.__get_longitude(self.__coord_corrections[1], lon)
            o = self.__coord_corrections[2] - yaw
        else:
            latitude = lat
            longitude = lon
            o = yaw - self.__coord_corrections[2]
            # o = yaw
        return latitude, longitude, o

    def calc_bearing(self, pointA, pointB):
        """
        Вычисляет азимут между двумя точками по формуле
       geodesic     θ = atan2(sin(Δlong).cos(lat2),
                    cos(lat1).sin(lat2) − sin(lat1).cos(lat2).cos(Δlong))
        Параметры:
        pointA: кортеж чисел с плавающей запятой: (широта, долгота) начальной точки
        pointB: кортеж с плавающей запятой: (широта, долгота) точки назначения
        Возвращаемое значение:
        float: азимут в градусах от севера
        """
        lat1 = math.radians(pointA[0])
        lat2 = math.radians(pointB[0])
        diffLong = math.radians(pointB[1] - pointA[1])

        x = math.sin(diffLong) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(diffLong))

        initial_bearing = math.atan2(x, y)

        # Convert from radians to degrees and normalize to 0-360
        initial_bearing = math.degrees(initial_bearing)
        compass_bearing = (initial_bearing + 360) % 360

        return compass_bearing

    def get_relative_coordinates(self, lat_goal, lon_goal, pos, pov_point):
        x, y, lat, lon, angle = pov_point[0], pov_point[1], pos[1], pos[0], pos[2]
        # Константа для перевода метров в пиксели
        meters_to_pixels = 15

        # Вычисление расстояния в метрах с помощью geopy
        start_coords = (lat, lon)
        goal_coords = (lon_goal, lat_goal)
        distance_meters = geodesic(start_coords, goal_coords).meters

        # Определение направления движения к целевой точке в глобальной системе координат
        bearing = self.calc_bearing(start_coords, goal_coords)
        bearing_radians = math.radians(bearing)

        # Учитываем угол поворота автомобиля
        relative_bearing = bearing_radians - angle

        # Перевод расстояния в пиксели
        distance_pixels = distance_meters * meters_to_pixels

        # Вычисляем смещение в координатах изображения
        delta_x = distance_pixels * math.sin(relative_bearing)
        delta_y = distance_pixels * math.cos(relative_bearing)

        # Вычисляем новые координаты на изображении
        goal_x = x + delta_x
        goal_y = y - delta_y  # Смещение вниз по оси Y уменьшает координату

        return (int(goal_x), int(goal_y))
    
    def get_relative_coordinates_f(self, lat_goal, lon_goal, pos, pov_point):
        x, y, lat, lon, angle = pov_point[0], pov_point[1], pos[1], pos[0], pos[2]
        # Константа для перевода метров в пиксели
        meters_to_pixels = 15

        # Вычисление расстояния в метрах с помощью geopy
        start_coords = (lat, lon)
        goal_coords = (lon_goal, lat_goal)
        
        distance_meters = geodesic(start_coords, goal_coords).meters

        # Определение направления движения к целевой точке в глобальной системе координат
        bearing = self.calc_bearing(start_coords, goal_coords)
        bearing_radians = math.radians(bearing)

        # Учитываем угол поворота автомобиля
        relative_bearing = bearing_radians - angle

        # Перевод расстояния в пиксели
        distance_pixels = distance_meters * meters_to_pixels
        # print(f'bearing_radians: {bearing_radians} {math.degrees(bearing_radians)} relative_bearing: {relative_bearing} {math.degrees(relative_bearing)} distance_pixels: {distance_pixels}')

        # Вычисляем смещение в координатах изображения
        delta_x = distance_pixels * math.sin(relative_bearing)
        delta_y = distance_pixels * math.cos(relative_bearing)

        # Вычисляем новые координаты на изображении
        goal_x = x + delta_x
        goal_y = y - delta_y  # Смещение вниз по оси Y уменьшает координату

        return (goal_x, goal_y)
    
    def get_coord_corrections(self):
        return self.__coord_corrections  

    def get_global_coordinates_from_ipm_coords_(self, relative_x, relative_y, pos, pov_point):
        start_lat, start_lon, start_angle, scale_x, scale_y, bev_orientation = pos[0], pos[1], pos[2], self.__coord_corrections[3], self.__coord_corrections[4], self.__coord_corrections[5]

        # Константа для перевода метров в пиксели
        meters_to_pixels = 15
 
        # Применяем обратное масштабирование (перевод пикселей в метры)
        unscaled_x = (relative_x - pov_point[0]) / meters_to_pixels
        unscaled_y = (relative_y + pov_point[1]) / meters_to_pixels
 
        # Применяем обратный поворот
        # start_angle = bev_orientation - start_angle
        # angle_rad = -start_angle  # Обратный угол -
 
        # rotated_x = unscaled_x * math.cos(angle_rad) - unscaled_y * math.sin(angle_rad)
        # rotated_y = unscaled_x * math.sin(angle_rad) + unscaled_y * math.cos(angle_rad)
 
        # Преобразуем относительные координаты в глобальные
        # latitude = self.__get_latitude_back(start_lat, rotated_y)
        # longitude = self.__get_longitude_back(start_lon, rotated_x, start_lat)
        rotated_distance = math.sqrt(unscaled_x ** 2 + unscaled_y ** 2)
        # rotated_angle = math.atan2(unscaled_y, unscaled_x) / math.pi * 180
        
        loc1_pt = distance(meters=rotated_distance).destination((start_lat, start_lon), bearing=-start_angle)

        return loc1_pt.latitude, loc1_pt.longitude
    
    def get_global_coordinates(self, local_x, local_y, pos, pov_point):
        """
        Преобразует точки в локальных координатах обратно в глобальные координаты.

        :param local_x: координата X в локальной системе
        :param local_y: координата Y в локальной системе
        :param pos: текущее положение (широта, долгота, угол поворота)
        :param pov_point: точка зрения (X, Y) в пикселях
        :return: кортеж глобальных координат (широта, долгота)
        """
        x, y, lat, lon, angle = pov_point[0], pov_point[1], pos[1], pos[0], pos[2]
        meters_to_pixels = 15

        # Смещение в пикселях
        delta_x = local_x - x
        delta_y = y - local_y  # Смещение вниз уменьшает координату

        # Учитываем угол поворота автомобиля
        relative_bearing = math.atan2(delta_x, delta_y)
        bearing_radians = relative_bearing + angle

        # Перевод пикселей в метры
        distance_pixels = math.sqrt(delta_x**2 + delta_y**2)
        distance_meters = distance_pixels / meters_to_pixels

        # Определение новой точки
        start_coords = (lat, lon)
        destination = geodesic(meters=distance_meters).destination(start_coords, math.degrees(bearing_radians))

        return (destination.longitude, destination.latitude)

    def __get_latitude_back(self, start_lat, delta_meters):
        m = (1 / ((2 * math.pi / 360) * self.__EARTH_RADIUS_KM)) / 1000
        return start_lat - (delta_meters * m)

    def __get_longitude_back(self, start_lon, delta_meters, start_lat):
        m = (1 / ((2 * math.pi / 360) * self.__EARTH_RADIUS_KM)) / 1000
        cos_lat = math.cos(start_lat * (math.pi / 180))
        return start_lon - (delta_meters * m) / cos_lat        
    
