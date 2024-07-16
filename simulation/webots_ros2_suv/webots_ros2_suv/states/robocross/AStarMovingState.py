from webots_ros2_suv.states.AbstractState import AbstractState
from webots_ros2_suv.lib.map_utils import calc_dist_point
from webots_ros2_suv.lib.config_loader import ConfigLoader
import math
import time

import pygame as pg
import numpy as np

screen_scale = 15

class AStarMovingState(AbstractState):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__( *args, **kwargs)
        self.runs = 0
        self.__cur_path_point = 1
        self.prev_target_angle = 0
        pg.font.init()
        self.sysfont = pg.font.SysFont("Arial", 20)
        self.params = {}
        self.road_offsets = []

        self.lane_coords = [
            (1, (52.00017417781055,55.819496316835284), (52.00016336515546,55.81918191164732), 2.0),
                (2, (52.00023570097983,55.819495394825935), (52.000227738171816,55.819173362106085), 2.0),
                (3, (52.00025984086096,55.81929858773947), (52.00153338722885,55.81928760744631), 2.0),
                (4, (52.00026176869869,55.81926195882261), (52.00153389014304,55.81925022415817), 2.0),
                (5, (52.000258415937424,55.819214936345816), (52.00163120403886,55.81920521333814), 2.0),
                (6, (52.000255482271314,55.819178810343146), (52.00162433087826,55.81916824914515), 2.0),
                (7, (52.00099811889231,55.81940218806267),(52.00099552050233,55.819310573861), 2.0),
                (8, (52.00106048025191,55.81930789165199), (52.001059809699655,55.819395147264004), 2.0),
                (9, (52.001574877649546,55.819640066474676), (52.00156431645155,55.819229101762176), 2.0),
                (10, (52.001626091077924,55.81922935321927), (52.00163631699979,55.81968960352242), 2.0),
                (11, (52.001538164913654,55.81953763961792), (52.000093292444944,55.819550547748804), 2.0),
                (12, (52.00153699144721,55.81949941813946), (52.00017778202891,55.81951509229839), 2.0),
                (13, (52.000130005180836,55.81953453831375), (52.00012690387666,55.8191829174757), 0.8),
                (14, (52.000158336013556,55.819165064021945), (52.00162877328694,55.81915215589106), 0.8),
                (15, (52.00167269445956,55.81916967406869), (52.00167923234403,55.81962615251541), 0.8)
        ]
        self.lidar_config = ConfigLoader("lidardata").data

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
        if not world_model.global_map:
            return (0, 0)
        
        points = [e['coordinates'] for e in world_model.global_map if 'moving' in e['name'] and 'seg_num' in e and int(e['seg_num']) == world_model.cur_path_segment][0]
        points = points[self.__cur_path_point:]
        world_model.gps_path = points

        self.__cur_path_point = world_model.cur_path_point

        dist = math.sqrt(calc_dist_point(points[0], world_model.get_current_position()))
        if dist < self.config['change_point_dist'] * 1.5:
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

        return [E, -N]

    def rotate_point(self, center, target, angle):
        return [math.cos(angle) * (target[0] - center[0]) - math.sin(angle) * (target[1] - center[1]) + center[0],
                math.sin(angle) * (target[0] - center[0]) + math.cos(angle) * (target[1] - center[1]) + center[1]]

    def move_screen(self, x, y):
        return [int(400 + x * screen_scale), int(400 + y * screen_scale)]

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

    def draw_box(self, screen, bounds, color):
        pg.draw.line(screen,
                    color,
                    self.move_screen(bounds[0][0], bounds[0][1]),
                    self.move_screen(bounds[1][0], bounds[1][1]))
        pg.draw.line(screen,
                    color,
                    self.move_screen(bounds[1][0], bounds[1][1]),
                    self.move_screen(bounds[2][0], bounds[2][1]))
        pg.draw.line(screen,
                    color,
                    self.move_screen(bounds[2][0], bounds[2][1]),
                    self.move_screen(bounds[3][0], bounds[3][1]))
        pg.draw.line(screen,
                    color,
                    self.move_screen(bounds[3][0], bounds[3][1]),
                    self.move_screen(bounds[0][0], bounds[0][1]))


    def on_event(self, event, world_model=None):
        world_model.software_state = 'Auto'
        self.runs = self.runs + 1
        # if world_model.traffic_light_state == 'red':
        #     return 'stop'

        # world_model.goal_point = (self.find_goal_point_x(world_model.ipm_image[10,:]), 10)
        # world_model.goal_point = self.find_next_goal_point(world_model)
        
        lat, lon, orientation = world_model.get_current_position() # Текущее месторасположение автомобиля
        orientation -= 1.5
        car_position = self.gps_to_rect(lat, lon)
        car_vector = [math.cos(orientation) * 2, math.sin(orientation) * 2]
        difference = None
        points = [e['coordinates'] for e in world_model.global_map if e['name'] == 'moving' and 'seg_num' in e and int(e['seg_num']) == world_model.cur_path_segment][0]
        path_source = []
        for point in points:
            path_source.append(self.gps_to_rect(point[0], point[1]))
        # TODO: add collecting of segment points
        if len(points) > 0 and len(self.road_offsets) == 0:
            self.road_offsets = np.zeros(len(points))
        while True:
            world_model.gps_path = points
            path_square_points = []
            
            path_square_points = path_source.copy()
            for i in range(1, min(self.__cur_path_point + 5,  len(path_source) - 1)):
                if self.road_offsets[i] == 0:
                    # self.logw(f"n = {i} {self.road_offsets[i]}")
                    difference_vector = world_model.coords_transformer.get_relative_coordinates_f(
                        points[i][0], 
                        points[i][1], 
                        pos=world_model.get_current_position(),
                        pov_point=[0, 0]
                    )
                    # difference_vector = [path_source[i][0] - car_position[0], path_source[i][1] - car_position[1]]
                    # difference_vector = self.rotate_point([0, 0], difference_vector, orientation - math.pi / 2)
                    for obstacle in world_model.obstacles:
                        x_dif = (-obstacle[8] + obstacle[9] + 1) * 15
                        y_dif = (obstacle[11] - obstacle[10] + 40) * 15
                        if x_dif < 1:
                            x_dif = 1
                        if y_dif < 1:
                            y_dif = 1
                        # obstacle_rect = pg.Rect(-obstacle[9], obstacle[10] - 10, x_dif, y_dif + 20)
                        obstacle_rect = pg.Rect((-obstacle[9] - 0.5) * 15, (obstacle[10] - 30) * 15, x_dif, y_dif)
                        
                        log_rect = [-obstacle[9], obstacle[10], x_dif, y_dif]
                        # self.logi(f"vector {difference_vector} log_rect {log_rect}")
                        
                        if obstacle_rect.collidepoint(difference_vector[0], -difference_vector[1]):
                            self.road_offsets[i] = 1
                            break
                    else:
                        if self.road_offsets[i - 1] == 1:
                            self.road_offsets[i] = 0.5
                            
                if self.road_offsets[i] != 0:
                    point_vector = [path_source[i + 1][0] - path_source[i + 1][0], path_source[i + 1][0] - path_source[i + 1][0]]
                    points_angle = self.AngleOfReference(point_vector) + math.pi / 2
                    path_square_points[i][0] += math.sin(points_angle) * 4.5 * self.road_offsets[i]
                    path_square_points[i][1] += math.cos(points_angle) * 4.5 * self.road_offsets[i]

            # path_source = path_source[self.__cur_path_point:]
            path_square_points = path_square_points[self.__cur_path_point:]

            if len(path_square_points) > 1:
                nearest_point = self.MedianVector(path_square_points[0], path_square_points[1], 0.75)
                difference = [nearest_point[0] - car_position[0], nearest_point[1] - car_position[1]]

                dist = math.sqrt(difference[0] ** 2 + difference[1] ** 2)
            else:
                break
            if dist < self.config['change_point_dist'] * 1.5:
                change_dist = self.config['change_point_dist'] * 1.5
                self.loge(f"point ++ {dist} < {change_dist}")
                self.__cur_path_point += 1
                world_model.cur_path_point = self.__cur_path_point
            else:
                break

        if len(path_square_points) > 1:            
            difference_angle = -self.AngleOfVectors(car_vector, difference)
            world_model.gps_car_turn_angle = float(min(1, max(-1, difference_angle / 90)))
            diff_angle = (self.prev_target_angle - world_model.gps_car_turn_angle) * 0.2
            world_model.gps_car_turn_angle = (world_model.gps_car_turn_angle + diff_angle)

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

        event = None
        zones = world_model.get_current_zones()

        speed = self.config['default_speed']
        zones_names = []
        for zone in zones:
            if zone['name'].startswith('speed'):
                speed = float(zone['name'].split('speed')[1])
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
            elif zone["name"] == "obstacle_stop":
                speed = 7
                if self.has_obstacle(world_model):
                    speed = 0
            elif zone["name"] == "to_gpsfollow":
                world_model.cur_path_segment += 1
                event = 'start_gps_follow'
            else:
                world_model.previous_zone = None
            zones_names.append(zone["name"])

        # if world_model.is_obstacle_before_path_point(filter_num=2, log=self.log):
        #     event = 'start_move'
        # if world_model.is_lane_road():
        #     event = 'start_lane_follow'
        self.params["zones"] = zones_names
        self.params["speed"] = speed
        self.params["traficlight"] = world_model.traffic_light_state
        self.params["current"] = self.__cur_path_point
        self.params["segment"] = world_model.cur_path_segment

        pg.event.get()
        world_model.sc.fill((0, 0, 0))

        pg.draw.line(world_model.sc, (255,0,0), self.move_screen(0, 0), self.move_screen(car_vector[0], car_vector[1]))
        if difference != None:
            pg.draw.line(world_model.sc, (0,255,0), self.move_screen(0, 0), self.move_screen(difference[0], difference[1]))
        for i in range(len(path_square_points)):
            color = (255, 0, 0)
            if self.road_offsets[i + 1] != 0:
                color = (0, 255, 0)
            pg.draw.circle(world_model.sc, color, self.move_screen(path_square_points[i][0] - car_position[0], path_square_points[i][1] - car_position[1]), 4)
        local_obstacles = world_model.obstacles.copy()
        for obstacle in local_obstacles:
            # x_offset = self.lidar_config["gps_shift_x"]
            # y_offset = self.lidar_config["gps_shift_x"]
            # obstacle[8] -= x_offset
            # obstacle[9] -= x_offset
            # obstacle[10] -= y_offset
            # obstacle[11] -= y_offset
            obstacle_points = [[-obstacle[8], obstacle[10]],
                               [-obstacle[9], obstacle[10]],
                               [-obstacle[9], obstacle[11]],
                               [-obstacle[8], obstacle[11]]]
            inflated_obstacles = [[-obstacle[8], obstacle[10] - 10],
                                  [-obstacle[9], obstacle[10] + 20],
                                  [-obstacle[9], obstacle[11] + 20],
                                  [-obstacle[8], obstacle[11] - 10]]
            
            rotated_obstacle = []
            rotated_inflated_obstacle = []
            for i in range(4):
                rotated_obstacle.append(self.rotate_point(center=[0, 0], target=[obstacle_points[i][0], obstacle_points[i][1]], angle=(orientation - (math.pi / 2))))
                rotated_inflated_obstacle.append(self.rotate_point(center=[0, 0], target=[inflated_obstacles[i][0], inflated_obstacles[i][1]], angle=(orientation - (math.pi / 2))))
            self.draw_box(world_model.sc, rotated_obstacle, (255, 255, 0))
            self.draw_box(world_model.sc, rotated_inflated_obstacle, (255, 0, 255))

        for lane in self.lane_coords:
            begin = self.gps_to_rect(lane[1][0], lane[1][1])
            end = self.gps_to_rect(lane[2][0], lane[2][1])
            begin_relative = [begin[0] - car_position[0], begin[1] - car_position[1]]
            end_relative = [end[0] - car_position[0], end[1] - car_position[1]]
            pg.draw.line(world_model.sc, (0,255,0), self.move_screen(begin_relative[0], begin_relative[1]), self.move_screen(end_relative[0], end_relative[1]))
            text_lane = self.sysfont.render(f"{lane[0]}", False, (255, 225, 255))
            world_model.sc.blit(text_lane, self.move_screen(begin_relative[0], begin_relative[1]))

        y = 10
        for k, v in self.params.items():
            text_reward = self.sysfont.render(f"{k}: {v}", False, (255, 0, 0))
            world_model.sc.blit(text_reward, (0, y))
            y += 20

        # for obstacle in world_model.obstacles:
        #     x_dif = (-obstacle[8] + obstacle[9]) * 15
        #     y_dif = (obstacle[11] - obstacle[10]) * 15
            
        #     obstacle_rect = pg.Rect((-obstacle[9]) * 15 + 400, 800 - (obstacle[10]) * 15 - y_dif, x_dif, y_dif)
        #     pg.draw.rect(world_model.sc, (255, 255, 255), obstacle_rect)

        # for i in range(1, min(self.__cur_path_point + 10,  len(path_source) - 1)):
        #     difference_vector = world_model.coords_transformer.get_relative_coordinates_f(
        #         points[i][0], 
        #         points[i][1], 
        #         pos=world_model.get_current_position(),
        #         pov_point=[0, 0]
        #     )
        #     # self.log(f"vector {i} {difference_vector}")
        #     pg.draw.circle(world_model.sc, (0, 255, 255), (-difference_vector[0] + 400, 800 + difference_vector[1]), 4)

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
            if obstacle[1] < self.config["obstacle_stop_distance"] and obstacle_angle < self.config["treshold_angle"]:
                return True
        return False

    def get_directrion(dir) -> int:
        return 1 if dir else -1