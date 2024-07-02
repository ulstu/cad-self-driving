from webots_ros2_suv.states.AbstractState import AbstractState
from webots_ros2_suv.lib.map_utils import calc_dist_point
import math
import time

import pygame as pg
import numpy as np


class GPSFollowState(AbstractState):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__( *args, **kwargs)
        self.runs = 0
        self.__cur_path_point = 1
        self.sc = pg.display.set_mode((800, 800))
        self.prev_target_angle = 0
        pg.font.init()
        self.sysfont = pg.font.SysFont("Arial", 20)
        self.params = {}

    def find_goal_point_x(self, arr, val=100):
        current_length, max_length = 0
        current_start, max_start = 0

        for i, element in enumerate(arr):
            if element == val:
                current_length += 1

                if current_length > max_length:
                    max_length = current_length
                    max_start = current_start
            else:
                current_length = 0
                current_start = i + 1

        if max_length > 0:
            max_end = max_start + max_length - 1
            return max_start + int((max_end - max_start) / 3 * 2)
        else:
            return 0

    def is_obstacle_near(self, world_model, x, y, obstacle_val, robot_radius):
        for i in range(x - robot_radius, x + robot_radius):
            for j in range(y - robot_radius, y + robot_radius):
                if world_model.ipm_image[j, i] != obstacle_val:
                    return False
        return True

    def find_next_goal_point(self, world_model):
        '''

        if not world_model.global_map:
            return (0, 0)

        points = [e['coordinates'] for e in world_model.global_map if e['name'] == 'moving' and 'seg_num' in e and int(e['seg_num']) == world_model.cur_path_segment][0]
        dists = []

        for point in points:
            dists.append(calc_dist_point(point, world_model.get_current_position()))
        self.__cur_path_point = world_model.cur_path_point

        if self.__cur_path_point < len(points) - 1:
            x, y = world_model.coords_transformer.get_relative_coordinates(points[self.__cur_path_point][0], points[self.__cur_path_point][1], world_model.get_current_position(), world_model.pov_point)
            dist = calc_dist_point(points[self.__cur_path_point], world_model.get_current_position())

            if dist < self.config['change_point_dist'] or (world_model.ipm_image.shape[1] > x and world_model.ipm_image.shape[0] > y and not self.is_obstacle_near(world_model, x, y, 100, 8)):
                self.__cur_path_point = self.__cur_path_point + 1
            self.log(f"[GPSFollowState] DIST {dist} CUR POINT: {self.__cur_path_point} SEG: {world_model.cur_path_segment}")
        else:
            self.__cur_path_point = len(points) - 1

        world_model.cur_path_point = self.__cur_path_point

        x, y = world_model.coords_transformer.get_relative_coordinates(
            points[self.__cur_path_point][0], 
            points[self.__cur_path_point][1], 
            pos=world_model.get_current_position(),
            pov_point=world_model.pov_point
        )

        return (x, y)

        '''

        if not world_model.global_map:
            return (0, 0)

        points = [e['coordinates'] for e in world_model.global_map if e['name'] == 'moving' and 'seg_num' in e and int(e['seg_num']) == world_model.cur_path_segment][0]
        points = points[self.__cur_path_point:]
        world_model.gps_path = points

        self.__cur_path_point = world_model.cur_path_point

        dist = math.sqrt(calc_dist_point(points[0], world_model.get_current_position()))
        if dist < self.config['change_point_dist']:
            self.__cur_path_point += 1

        world_model.cur_path_point = self.__cur_path_point

        x, y = world_model.coords_transformer.get_relative_coordinates(
            points[self.__cur_path_point][0], 
            points[self.__cur_path_point][1], 
            pos=world_model.get_current_position(),
            pov_point=world_model.pov_point
        )

        return (x, y)

    def gps_to_rect(self, dLon, dLat):
        # Номер зоны Гаусса-Крюгера
        zone = int(dLon / 6.0 + 1)

        # Параметры эллипсоида Красовского
        a = 6378245.0  # Большая (экваториальная) полуось
        b = 6356863.019  # Малая (полярная) полуось
        e2 = (a ** 2 - b ** 2) / a ** 2  # Эксцентриситет
        n = (a - b) / (a + b)  # Приплюснутость

        # Параметры зоны Гаусса-Крюгера
        F = 1.0  # Масштабный коэффициент
        Lat0 = 0.0  # Начальная параллель (в радианах)
        Lon0 = (zone * 6 - 3) * math.pi / 180  # Центральный меридиан (в радианах)
        N0 = 0.0  # Условное северное смещение для начальной параллели
        E0 = zone * 1e6 + 500000.0  # Условное восточное смещение для центрального меридиана

        # Перевод широты и долготы в радианы
        Lat = dLat * math.pi / 180.0
        Lon = dLon * math.pi / 180.0

        # Вычисление переменных для преобразования
        v = a * F * (1 - e2 * (math.sin(Lat) ** 2)) ** -0.5
        p = a * F * (1 - e2) * (1 - e2 * (math.sin(Lat) ** 2)) ** -1.5
        n2 = v / p - 1
        M1 = (1 + n + 5.0 / 4.0 * n ** 2 + 5.0 / 4.0 * n ** 3) * (Lat - Lat0)
        M2 = (3 * n + 3 * n ** 2 + 21.0 / 8.0 * n ** 3) * math.sin(Lat - Lat0) * math.cos(Lat + Lat0)
        M3 = (15.0 / 8.0 * n ** 2 + 15.0 / 8.0 * n ** 3) * math.sin(2 * (Lat - Lat0)) * math.cos(2 * (Lat + Lat0))
        M4 = 35.0 / 24.0 * n ** 3 * math.sin(3 * (Lat - Lat0)) * math.cos(3 * (Lat + Lat0))
        M = b * F * (M1 - M2 + M3 - M4)
        I = M + N0
        II = v / 2 * math.sin(Lat) * math.cos(Lat)
        III = v / 24 * math.sin(Lat) * (math.cos(Lat)) ** 3 * (5 - (math.tan(Lat) ** 2) + 9 * n2)
        IIIA = v / 720 * math.sin(Lat) * (math.cos(Lat) ** 5) * (61 - 58 * (math.tan(Lat) ** 2) + (math.tan(Lat) ** 4))
        IV = v * math.cos(Lat)
        V = v / 6 * (math.cos(Lat) ** 3) * (v / p - (math.tan(Lat) ** 2))
        VI = v / 120 * (math.cos(Lat) ** 5) * (5 - 18 * (math.tan(Lat) ** 2) + (math.tan(Lat) ** 4) + 14 * n2 - 58 * (math.tan(Lat) ** 2) * n2)

        # Вычисление северного и восточного смещения (в метрах)
        N = I + II * (Lon - Lon0) ** 2 + III * (Lon - Lon0) ** 4 + IIIA * (Lon - Lon0) ** 6
        E = E0 + IV * (Lon - Lon0) + V * (Lon - Lon0) ** 3 + VI * (Lon - Lon0) ** 5

        return [E / 5, -N / 5]

    def move_screen(self, x, y):
        return [int(400 + x * 60), int(400 + y * 60)]

    def AngleOfReference(self, v):
        return self.NormalizeAngle(math.atan2(v[1], v[0]) / math.pi * 180)

    def AngleOfVectors(self, first, second):
        return self.NormalizeAngle(self.AngleOfReference(first) - self.AngleOfReference(second))

    def NormalizeAngle(self, angle):
        if angle > -180:
            turn = -360
        else:
            turn = 360
        while not (angle > -180 and angle <= 180):
            angle += turn
        return angle
    
    def MedianVector(self, first, second, k):
        return [first[0] * k + second[0] * (1 - k), first[1] * k + second[1] * (1 - k)]


    def on_event(self, event, world_model=None):
        world_model.software_state = 'Auto'
        self.runs = self.runs + 1
        self.log("gps follow state")
        # if world_model.traffic_light_state == 'red':
        #     return 'stop'

        # world_model.goal_point = (self.find_goal_point_x(world_model.ipm_image[10,:]), 10)
        # world_model.goal_point = self.find_next_goal_point(world_model)
        
        lat, lon, orientation = world_model.get_current_position() # Текущее месторасположение автомобиля
        orientation -= 1.5
        car_position = self.gps_to_rect(lat, lon)
        car_vector = [math.cos(orientation) * 2, math.sin(orientation) * 2]
        difference = None

        while True:
            points = [e['coordinates'] for e in world_model.global_map if e['name'] == 'moving' and 'seg_num' in e and int(e['seg_num']) == world_model.cur_path_segment][0]
            points_offset = self.__cur_path_point
            points = points[points_offset:] # Удаляем из него те точки, которые были достигнуты автомобилем
            world_model.gps_path = points

            path_square_points = []
            for point in points:
                path_square_points.append(self.gps_to_rect(point[0], point[1]))

            if len(path_square_points) > 1:
                nearest_point = self.MedianVector(self.gps_to_rect(points[0][0], points[0][1]), self.gps_to_rect(points[1][0], points[1][1]), 0.75)
                difference = [nearest_point[0] - car_position[0], nearest_point[1] - car_position[1]]

                dist = math.sqrt(difference[0] ** 2 + difference[1] ** 2)

                conf = self.config['change_point_dist']
            else:
                break
            if dist < self.config['change_point_dist']:
                self.__cur_path_point += 1
                world_model.cur_path_point = self.__cur_path_point
            else:
                break

        if len(path_square_points) > 1:            
            difference_angle = -self.AngleOfVectors(car_vector, difference)
            world_model.gps_car_turn_angle = float(min(1, max(-1, difference_angle / 45)))
            diff_angle = (self.prev_target_angle - world_model.gps_car_turn_angle) * 0.2
            world_model.gps_car_turn_angle = world_model.gps_car_turn_angle + diff_angle
            self.logi(f"diff = {self.prev_target_angle - world_model.gps_car_turn_angle} new = {world_model.gps_car_turn_angle}, old ={self.prev_target_angle}")
            
            self.params["diff"] = self.prev_target_angle - world_model.gps_car_turn_angle
            self.params["new"] = world_model.gps_car_turn_angle
            self.params["old"] = self.prev_target_angle
            self.params["prevzone"] = world_model.previous_zone
            self.params["hardware_state"] = world_model.hardware_state
            self.params["software_state"] = world_model.software_state
            self.params["has_obstacle"] = self.has_obstacle(world_model)

            self.prev_target_angle = world_model.gps_car_turn_angle
        else:
            world_model.gps_car_turn_angle = 0.0
        pg.event.get()
        self.sc.fill((0, 0, 0))

        pg.image.frombuffer(world_model.ipm_colorized.tostring(), world_model.ipm_colorized.shape[1::-1], "BGR")

        pg.draw.line(self.sc, (255,0,0), self.move_screen(0, 0), self.move_screen(car_vector[0], car_vector[1]))
        if difference != None:
            pg.draw.line(self.sc, (0,255,0), self.move_screen(0, 0), self.move_screen(difference[0], difference[1]))
        for point in path_square_points:
            pg.draw.circle(self.sc, (255,0,0), self.move_screen(point[0] - car_position[0], point[1] - car_position[1]), 4)
        y = 10
        

        event = None
        zones = world_model.get_current_zones()

        # self.logi(f'ipm {world_model.ipm_colorized.shape}')

        speed = self.config['default_speed']
        zones_names = []
        for zone in zones:
            if zone['name'] == 'turn':
                world_model.cur_turn_polygon = zone['coordinates'][0]
                self.__cur_path_point = 0
                event = 'turn'
            elif zone['name'] == 'terminal' and self.__cur_path_point > 10:
                self.logi(f"{world_model.previous_zone}")
                self.logi(f"Inside terminal")
                world_model.previous_zone = zone['name']
                speed = 0
                world_model.cur_path_point = 0
                self.__cur_path_point = 0
                event = 'pause'
            elif zone['name'] == 'traffic_light':
                if world_model.traffic_light_state == 'red':
                    speed = 0
                    # self.__cur_path_point = 0
                    event = 'stop'
            elif zone['name'] == 'crosswalk':
                if world_model.pedestrian_on_crosswalk:
                    speed = 0
                    # self.__cur_path_point = 0
                    event = 'stop'
            elif zone['name'] == 'stop':
                self.__cur_path_point = 0
                event = 'stop'
            elif zone['name'].startswith('speed'):
                speed = float(zone['name'].split('speed')[1])
            elif zone["name"] == "obstacle_stop":
                if self.has_obstacle(world_model):
                    speed = 0
            else:
                self.logi(f"{world_model.previous_zone} go out")
                world_model.previous_zone = None
            zones_names.append(zone["name"])

        # if world_model.is_obstacle_before_path_point(filter_num=2, log=self.log):
        #     event = 'start_move'
        # if world_model.is_lane_road():
        #     event = 'start_lane_follow'
        self.params["zones"] = zones_names

        for k, v in self.params.items():
            text_reward = self.sysfont.render(f"{k}: {v}", False, (255, 0, 0))
            self.sc.blit(text_reward, (0, y))
            y += 15
        pg.display.update()

        if event:
            world_model.path = None
        world_model.set_speed(speed)
        self.drive(world_model, speed=speed)
        return event
    
    def has_obstacle(self, world_model):
        for obstacle in world_model.obstacles:
            mean_obstacle_position = [(obstacle[8] + obstacle[9]) / 2, (obstacle[10] + obstacle[11]) / 2]
            obstacle_angle = abs(self.AngleOfReference(mean_obstacle_position))
            self.logi(f"position {mean_obstacle_position} angle {obstacle_angle}")
            if obstacle[1] < self.config["obstacle_stop_distance"] and obstacle_angle < self.config["treshold_angle"]:
                return True
        return False

