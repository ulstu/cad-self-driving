from AbstractWorker import AbstractWorker
import cv2
import numpy as np
import traceback
from fastseg.image import colorize, blend
from webots_ros2_suv.lib.timeit import timeit
from webots_ros2_suv.lib.rrt_star_reeds_shepp import RRTStarReedsShepp
from webots_ros2_suv.lib.rrt_star_direct import rrt_star, RRTTreeNode, visualize_path
from webots_ros2_suv.lib.rrt_star import RRTStar
import matplotlib.pyplot as plt


class PathPlanningWorker(AbstractWorker):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__( *args, **kwargs)

    def plan_rrt_star(self, world_model):
        start = [world_model.pov_point[0], world_model.pov_point[1], np.deg2rad(0.0)]
        goal = [world_model.goal_point[0],world_model.goal_point[1], np.deg2rad(0.0)]
        max_iter = 200
        step_size = 10
        rand_area=[300.0, 500.0, 100, 200]

        rrt_star = RRTStar(
            start=start,
            goal=goal,
            rand_area=rand_area,
            obstacle_list=world_model.ipm_image,
            expand_dis=1,
            robot_radius=0.0,
            path_resolution=1.0,
            goal_sample_rate=20,
            max_iter=200,
            connect_circle_dist=50.0,
            search_until_max_iter=False)
        path = rrt_star.planning(animation=True)


        if path:
            world_model.path = [(int(p[0]), int(p[1])) for p in path]

    def plan_rrt_star_direct(self, world_model):
        start_node = RRTTreeNode(world_model.pov_point[0], world_model.pov_point[1])
        goal_node = RRTTreeNode(world_model.goal_point[0],world_model.goal_point[1])
        
        max_iterations = 200
        step_size = 30
        car_length = 30  # Длина автомобиля
        car_width = 30   # Ширина автомобиля
        
        world_model.path, nodes = rrt_star(start_node, goal_node, world_model.ipm_image, max_iterations, step_size, car_length, car_width)

    def plan_path(self, world_model):
        if not world_model.goal_point:
            return world_model

        world_model.path = None
        # self.plan_rrt_star(world_model)
        # self.plan_rrt_star_reeds_shepp(world_model)
        self.plan_rrt_star_direct(world_model)

        #super().log(f'PATH: {world_model.path}')
        return world_model

    def on_event(self, event, scene=None):
        print("Emergency State")
        return None

    #@timeit
    def on_data(self, world_model):
        try:
            # super().log(f"PathPlanningWorker {str(world_model)}")
            world_model = self.plan_path(world_model)

        except  Exception as err:
            super().error(''.join(traceback.TracebackException.from_exception(err).format()))

        return world_model
