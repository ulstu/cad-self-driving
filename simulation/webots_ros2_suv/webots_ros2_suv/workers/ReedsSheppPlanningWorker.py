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


class ReedsSheppPlanningWorker(AbstractWorker):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__( *args, **kwargs)

    def plan_rrt_star_reeds_shepp(self, world_model):
        start = [world_model.pov_point[0], world_model.pov_point[1], np.deg2rad(0.0)]
        goal = [world_model.goal_point[0],world_model.goal_point[1], np.deg2rad(0.0)]
        max_iter = 200
        step_size = 0.3
        rand_area=[300.0, 500.0, 100, 200]
        rrt_star_reeds_shepp = RRTStarReedsShepp(start, goal,
                                                world_model.ipm_image,
                                                rand_area=rand_area, step_size=step_size, max_iter=max_iter)
        path = rrt_star_reeds_shepp.planning(animation=False)
        self.log(f'REEDS_SHEPP PATH: {path}')
        if path:
            world_model.path = [(int(p[0]), int(p[1])) for p in path]
    
    def plan_path(self, world_model):
        if not world_model.goal_point:
            return world_model

        world_model.path = None
        self.plan_rrt_star_reeds_shepp(world_model)

        super().log(f'PATH: {world_model.path}')
        return world_model

    def on_event(self, event, scene=None):
        print("Emergency State")
        return None

    #@timeit
    def on_data(self, world_model):
        try:
            world_model = self.plan_path(world_model)
            world_model.draw_scene()
        except  Exception as err:
            super().error(''.join(traceback.TracebackException.from_exception(err).format()))

        return world_model
