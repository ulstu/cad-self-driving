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
from webots_ros2_suv.lib.map_utils import is_point_in_polygon, calc_dist_point
from typing import List

class WorldModel(object):
    '''
    Класс, моделирующий параметры внешней среды в привязке к глобальной карте.
    '''
    def __init__(self, vehicles):
        self.__vehicles = vehicles
        self.__car_models = {}
        for vehicle in self.__vehicles:
            self.__car_models[vehicle] = CarModel()
        self.coords_transformer = CoordsTransformer()

    def load_map(self, mapyaml):
        self.global_map = []
        for f in mapyaml['features']:
            self.global_map.append({
                'name': f['properties']['id'].replace('_point', ''),
                'type': f['geometry']['type'],
                'coordinates': f['geometry']['coordinates'],
                'seg_num': f['properties'].get('seg_num', 0)
            })

    def get_current_position(self, vehicle):
        return self.__car_models[vehicle].get_position()

    def fill_params(self):
        pos = self.__car_model.get_position()
        self.params["cur_point"] = self.cur_path_point
        self.params["cur_path_segment"] = self.cur_path_segment
        self.params['lat'] = pos[0]
        self.params['lon'] = pos[1]
        self.params['angle'] = pos[2]

    def draw_scene(self, log=print):
        pass

    def get_speed(self, vehicle):
        return self.__car_models[vehicle].get_speed(vehicle)
    
    def set_speed(self, speed, vehicle):
        self.__car_models[vehicle].set_speed(speed)

    def set_rgb_image(self, image, vehicle):
        self.__car_models[vehicle].rgb_image = image

    def get_rgb_image(self, vehicle):
        return self.__car_models[vehicle].rgb_image

    def update_car_pos(self, lat, lon, orientation, vehicle):        
        self.__car_models[vehicle].update(lat=lat, lon=lon, orientation=orientation)

    def get_current_zones(self, vehicle):
        lat, lon, o = self.get_current_position(vehicle)
        zones = []
        for p in self.global_map:
            if p['type'] == 'Polygon':
                if is_point_in_polygon(lat, lon, p['coordinates'][0]): # and self.cur_path_point > 2:
                    zones.append(p)
        return zones

    # def drive(self, vehicle):
    #     self.__car_models[vehicle].