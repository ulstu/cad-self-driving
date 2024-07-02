from AbstractWorker import AbstractWorker
import cv2
import os
import yaml
import pathlib
import numpy as np
import traceback
import math
from fastseg.image import colorize, blend
from ament_index_python.packages import get_package_share_directory
from filterpy.kalman import KalmanFilter
from webots_ros2_suv.lib.timeit import timeit
from webots_ros2_suv.lib.rrt_star_reeds_shepp import RRTStarReedsShepp
from webots_ros2_suv.lib.rrt_star_direct import rrt_star, RRTTreeNode, visualize_path
from webots_ros2_suv.lib.rrt_star import RRTStar
import matplotlib.pyplot as plt
from webots_ros2_suv.lib.a_star import astar, kalman_filter_path, smooth_path_with_dubins, bezier_curve
from threading import Thread
from geopy.distance import geodesic
from webots_ros2_suv.lib.map_utils import is_point_in_polygon, calc_dist_point


class PathPlanningWorker(AbstractWorker):

    def __init__(self, *args, **kwargs) -> None:
        super().__init__( *args, **kwargs)
        package_dir = get_package_share_directory('webots_ros2_suv')
        config_path = os.path.join(package_dir,
                                    pathlib.Path(os.path.join(package_dir, 'config', 'global_coords.yaml')))
        self.turning_radius = 0.1
        self.process_noise_scale = 0.05 
        self.robot_radius=10
        self.step_size=10
        self.sample_size=5
        self.buffer = None
        with open(config_path) as file:
            config = yaml.full_load(file)
            self.turning_radius = config['dubins_turning_radius']
            self.process_noise_scale = config['kalman_path_planning_noise_scale']
            self.robot_radius = config['robot_radius']
            self.step_size = config['a_star_step_size']
            self.sample_size = config['dubins_sample_size']
            self.min_path_points_dist = config['min_path_points_dist']
            self.bezier_num_points = config['bezier_num_points']


    def plan_a_star(self, world_model):
        world_model.path = astar(world_model.pov_point, 
                                 world_model.goal_point, 
                                 world_model.ipm_image, 
                                 self.robot_radius, 
                                 self.step_size,
                                 super().log)
        
        world_model.gps_path = []
        if world_model.path != None and len(world_model.path) > 0:
            for point in world_model.path:
                world_model.gps_path.append(world_model.coords_transformer.get_global_coordinates(point[0], point[1], world_model.get_current_position(), world_model.pov_point))
            world_model.gps_path = kalman_filter_path(world_model.gps_path)
        # if not world_model.path and self.buffer:
            # world_model.path = self.buffer
            # world_model.params['final_path_len'] = len(world_model.path)
        #     # world_model.path = self.linear(world_model.pov_point, world_model.goal_point, 8)
        #     # world_model.path = bezier_curve(world_model.path, 20)
        #     world_model.params['init_path_len'] = "-"
        #     # if world_model.path:
        #         # world_model.params['linear_path_len'] = len(world_model.path)
                
        #         # world_model.params['final_path_len'] = len(world_model.path)
        #     # else:
        #     world_model.path = self.linear(world_model.pov_point, world_model.goal_point, 8)
        #     world_model.params['linear_path_len'] = len(world_model.path)
        #     world_model.path = bezier_curve(world_model.path, 20)
        #     world_model.params['final_path_len'] = len(world_model.path)
        if world_model.path:
            world_model.params['linear_path_len'] = '-'
            world_model.params['init_path_len'] = len(world_model.path)
            # world_model.path = kalman_filter_path(world_model.path, self.process_noise_scale)
            world_model.path = self.filter_coordinates(world_model.path, self.min_path_points_dist, 10)
            # world_model.path = smooth_path_with_dubins(world_model.path, self.turning_radius, world_model.ipm_image, self.sample_size, super().log)
            world_model.path = bezier_curve(world_model.path, self.bezier_num_points)
            self.buffer = world_model.path
        else:
            world_model.path = self.linear(world_model.pov_point, world_model.goal_point, 8)
            world_model.params['linear_path_len'] = len(world_model.path)

            world_model.path = bezier_curve(world_model.path, 20)
            world_model.params['init_path_len'] = "-"
            world_model.params['final_path_len'] = len(world_model.path)
    def linear(self, p1, p2, n):
        return [p1, p2]
    
    def calculate_angle(self, p1, p2, p3):
        """Calculate the angle between the line segments (p1p2) and (p2p3)."""
        v1 = (p1[0] - p2[0], p1[1] - p2[1])
        v2 = (p3[0] - p2[0], p3[1] - p2[1])
        angle = math.atan2(v2[1], v2[0]) - math.atan2(v1[1], v1[0])
        return abs(angle)

    def filter_coordinates(self, coordinates, min_path_points_dist, angle_threshold_degrees):
        if not coordinates:
            return []

        angle_threshold = math.radians(angle_threshold_degrees)
        filtered_coords = [coordinates[0]]

        for i in range(1, len(coordinates)):
            if math.sqrt((filtered_coords[-1][0] - coordinates[i][0]) ** 2 + (filtered_coords[-1][1] - coordinates[i][1]) ** 2) >= min_path_points_dist:
                if len(filtered_coords) < 2 or self.calculate_angle(filtered_coords[-2], filtered_coords[-1], coordinates[i]) >= angle_threshold:
                    filtered_coords.append(coordinates[i])

        return filtered_coords

    def check_path(self, world_model):
        n = 0
        for i in range(2, len(world_model.gps_path)):
            point = world_model.gps_path[i]

            relative_point = world_model.coords_transformer.get_relative_coordinates(point[0], 
                                                                                     point[1], 
                                                                                     pos=world_model.get_current_position(),
                                                                                     pov_point=world_model.pov_point)
            
            sub_ipm = world_model.ipm_image[relative_point[1] - 10:relative_point[1] + 10, relative_point[0] - 10:relative_point[0] + 10]
            if sub_ipm.mean(axis=0).mean(axis=0) < 90:
                return False
            n += 1
        return True
    
    def find_next_goal_point(self, world_model):
        if not world_model.global_map:
            return (0, 0)
        points = [e['coordinates'] for e in world_model.global_map if e['name'] == 'moving' and 'seg_num' in e and int(e['seg_num']) == world_model.cur_path_segment]

        points = points[0]

        dists = []
        for p in points:
            dists.append(calc_dist_point(p, world_model.get_current_position()))
        self.__cur_path_point = world_model.cur_path_point
        while True:
            if self.__cur_path_point < len(points) - 1:
                x, y = world_model.coords_transformer.get_relative_coordinates(points[self.__cur_path_point][0], points[self.__cur_path_point][1], world_model.get_current_position(), world_model.pov_point)
                dist = calc_dist_point(points[self.__cur_path_point], world_model.get_current_position())
                world_model.params["point_dist"] = dist
                if dist < self.config['change_point_dist']:
                    # or (world_model.ipm_image.shape[1] > x and world_model.ipm_image.shape[0] > y and not self.is_obstacle_near(world_model, x, y, 100, 8)):
                    self.__cur_path_point = self.__cur_path_point + 1
                    if (world_model.goal_point):
                        self.plan_a_star(world_model)
                else:
                    break
                world_model.params['dist_point'] = dist
                # self.log(f"[MovingState] DIST {dist} CUR POINT: {self.__cur_path_point} SEG: {world_model.cur_path_segment}")
            else:
                self.__cur_path_point = len(points) - 1
                break
        world_model.cur_path_point = self.__cur_path_point

        x, y = world_model.coords_transformer.get_relative_coordinates(points[self.__cur_path_point][0], 
                                                                       points[self.__cur_path_point][1], 
                                                                       pos=world_model.get_current_position(),
                                                                       pov_point=world_model.pov_point)
        return (x, y)

    def plan_path(self, world_model):

        world_model.goal_point = self.find_next_goal_point(world_model)

        if not world_model.goal_point:
            return world_model

        if world_model.gps_path == None:
            self.plan_a_star(world_model)
            self.logw("Path is empty")
        elif len(world_model.gps_path) == 0:
            self.plan_a_star(world_model)
            self.logw("Path is empty")
        elif not self.check_path(world_model):
            self.plan_a_star(world_model)
            self.logw("Path stucked")
        
        # self.logi(f"len {len(world_model.path)}")

        if len(world_model.gps_path) == 0:
            self.logw(f'Path is empty')
            
        return world_model

    def on_event(self, event, scene=None):
        print("Emergency State")
        return None

    #@timeit
    def on_data(self, world_model):
        # try:
        # thread = Thread(target = self.plan_path, args = (world_model,))
        # thread.start()
        # world_model = self.plan_path(world_model)
        # pass

        # except  Exception as err:
        #     super().error(''.join(traceback.TracebackException.from_exception(err).format()))

        return world_model
