from webots_ros2_suv.states.AbstractState import AbstractState

import pygame as pg
import math
import numpy as np
import cv2

class AStarMovingState(AbstractState):

    def __init__(self, *args, **kwargs) -> None:
        super().__init__( *args, **kwargs)
        self.runs = 0
        self.__cur_path_point = 1
        self.__cur_gps_point = 1
        self.params = {}
        pg.font.init()
        self.sysfont = pg.font.SysFont("Arial", 20)


    def find_goal_point_x(self, arr, val=100):
        max_length = 0
        max_start = 0
        current_length = 0
        current_start = 0

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

    def move_screen(self, x, y):
        return [int(400 + x * 15), int(400 + y * 15)]


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

        return [E, -N]
    
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
        self.runs = self.runs + 1
        # if world_model.traffic_light_state == "red":
        #     return "stop"
        #world_model.goal_point = (self.find_goal_point_x(world_model.ipm_image[10,:]), 10)
        # self.log(f'CUR POS: {lat} {lon} {world_model.goal_point}')

        lat, lon, orientation = world_model.get_current_position() # Текущее месторасположение автомобиля
        orientation -= 1.5
        car_position = self.gps_to_rect(lat, lon)
        car_vector = [math.cos(orientation) * 2, math.sin(orientation) * 2]
        difference = None

        if world_model.cur_path_point > self.__cur_path_point:
            self.__cur_gps_point = 1
            self.__cur_path_point = world_model.cur_path_point

        while True:
            points = []
            if world_model.gps_path != None:
                points = world_model.gps_path[self.__cur_gps_point:]

            path_square_points = []
            for point in points:
                path_square_points.append(self.gps_to_rect(point[0], point[1]))

            if len(path_square_points) > 1:
                nearest_point = self.MedianVector(self.gps_to_rect(points[0][0], points[0][1]), self.gps_to_rect(points[1][0], points[1][1]), 0.7)
                difference = [nearest_point[0] - car_position[0], nearest_point[1] - car_position[1]]

                dist = math.sqrt(difference[0] ** 2 + difference[1] ** 2)

                conf = self.config['change_point_dist']
            else:
                break

            if dist < self.config['change_point_dist']:
                self.__cur_gps_point += 1
            else:
                break

        if len(path_square_points) > 1:            
            difference_angle = -self.AngleOfVectors(car_vector, difference)
            world_model.gps_car_turn_angle = float(min(1, max(-1, difference_angle / 60)))
        else:
            world_model.gps_car_turn_angle = 0.0

        pg.event.get()
        world_model.sc.fill((0, 0, 0))

        # bev_image = pg.image.frombuffer(world_model.ipm_colorized_lines.tostring(), world_model.ipm_colorized_lines.shape[1::-1], "BGR")
        bev_image = pg.surfarray.make_surface(cv2.transpose(world_model.ipm_colorized_lines))
        world_model.sc.blit(bev_image, (0,0))

        # pg.draw.line(self.sc, (255,0,0), self.move_screen(0, 0), self.move_screen(car_vector[0], car_vector[1]))
        # if difference != None:
        #     pg.draw.line(self.sc, (0,255,0), self.move_screen(0, 0), self.move_screen(difference[0], difference[1]))
        # for point in path_square_points:
        #     pg.draw.circle(self.sc, (255,0,0), self.move_screen(point[0] - car_position[0], point[1] - car_position[1]), 4)
        for point in points:
            bev_position = world_model.coords_transformer.get_relative_coordinates(point[0], 
                                                                                   point[1], 
                                                                                   pos=world_model.get_current_position(),
                                                                                   pov_point=world_model.pov_point)
            pg.draw.circle(world_model.sc, (255,255,0, 0.5), bev_position, 10)
        
        

        event = None
        zones = world_model.get_current_zones()
        
        # default speed
        speed = 10
        for z in zones:
            if z['name'] == 'turn':
                world_model.cur_turn_polygon = z['coordinates'][0]
                self.__cur_path_point = 0
                event = "turn"
            elif z['name'] == 'stop':
                self.__cur_path_point = 0
                event = "stop"
            elif z['name'].startswith("speed"):
                speed = float(z['name'].split('speed')[1])
        # if not world_model.is_obstacle_before_path_point(filter_num=10, log=self.log):
        #     event = "start_gps_follow"
        # if world_model.is_lane_road():
        #     event = "start_lane_follow"
        self.params["cur_gps_point"] = self.__cur_gps_point
        self.params["cur_point"] = self.__cur_path_point
        y = 10
        for k, v in self.params.items():
            text_reward = self.sysfont.render(f"{k}: {v}", False, (255, 0, 0))
            world_model.sc.blit(text_reward, (0, y))
            y += 15

        pg.display.update()

        if event:
            world_model.path = None
        world_model.set_speed(speed)
        self.drive(world_model, speed=speed)
        return event