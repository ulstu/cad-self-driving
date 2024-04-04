from AbstractWorker import AbstractWorker
import cv2
import numpy as np
import traceback
from fastseg.image import colorize, blend
from webots_ros2_suv.lib.timeit import timeit
from webots_ros2_suv.lib.rrt_star_reeds_shepp import RRTStarReedsShepp
from webots_ros2_suv.lib.reeds_shepp_path_planning import reeds_shepp_path_planning, plot_arrow
from webots_ros2_suv.lib.rrt_star_direct import rrt_star, RRTTreeNode, visualize_path
from webots_ros2_suv.lib.rrt_star import RRTStar
import matplotlib.pyplot as plt
from webots_ros2_suv.lib.map_utils import calc_dist_point


class ReedsSheppPlanningWorker(AbstractWorker):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__( *args, **kwargs)

    def plan_rrt_star_reeds_shepp(self, world_model):
        start = [world_model.pov_point[0], world_model.pov_point[1], np.deg2rad(0.0)]
        goal = [world_model.goal_point[0],world_model.goal_point[1], np.deg2rad(0.0)]
        max_iter = 100
        step_size = 2
        rand_area=[0.0, 500.0, 100, 200]
        rrt_star_reeds_shepp = RRTStarReedsShepp(start, goal,
                                                world_model.ipm_image,
                                                rand_area=rand_area, step_size=step_size, max_iter=max_iter)
        path = rrt_star_reeds_shepp.planning(animation=False)
        if path and len(path) > 1:
            world_model.path = [(int(p[0]), int(p[1])) for p in path]
            self.log(f'REEDS_SHEPP  len: {len(world_model.path)} PATH: {world_model.path}')
        elif not path and world_model.path:
            # find closest point to vehicle
            pos = world_model.get_current_position()
            world_model.gps_path = [world_model.coords_transformer.get_global_coordinates_from_ipm_coords(p[0], p[1], pos) for p in world_model.path]
            gc = world_model.get_current_position()
            min_index = min(range(len(world_model.gps_path)), key=lambda index: calc_dist_point(gc, world_model.gps_path[index]))
            self.log(f'REEDS_SHEPP reduce: {min_index}')
            world_model.gps_path = world_model.gps_path[min_index:]
            world_model.path = world_model.path[min_index:]
            self.log(f'REEDS_SHEPP  len: {len(world_model.path)} PATH: {world_model.path}')
    
    def plan_reeds_shepp(self, world_model):
        start = [world_model.pov_point[0], world_model.pov_point[1], np.deg2rad(0.0)]
        goal = [world_model.goal_point[0],world_model.goal_point[1], np.deg2rad(0.0)]
        max_iter = 100
        step_size = 2
        rand_area=[0.0, 500.0, 100, 200]
        # rrt_star_reeds_shepp = RRTStarReedsShepp(start, goal,
        #                                         world_model.ipm_image,
        #                                         rand_area=rand_area, step_size=step_size, max_iter=max_iter)


        start_x = start[0]  # [m]
        start_y = start[1]  # [m]
        start_yaw = np.deg2rad(0.0)  # [rad]

        end_x = goal[0]  # [m]
        end_y = goal[1]  # [m]
        end_yaw = np.deg2rad(180.0)  # [rad]

        curvature = 1.5
        step_size = 0.5

        xs, ys, yaws, modes, lengths = reeds_shepp_path_planning(start_x, start_y,
                                                                start_yaw, end_x,
                                                                end_y, end_yaw,
                                                                curvature,
                                                                step_size)
        show_animation = False
        if show_animation:  # pragma: no cover
            plt.cla()
            plt.plot(xs, ys, label="final course " + str(modes))
            print(f"{lengths=}")

            # plotting
            plot_arrow(start_x, start_y, start_yaw)
            plot_arrow(end_x, end_y, end_yaw)

            plt.legend()
            plt.grid(True)
            plt.axis("equal")
            plt.show()
        # path = rrt_star_reeds_shepp.planning(animation=True)
        if xs and len(xs) > 1:
            world_model.path = [(int(xs[i]), int(ys[i])) for i in range(len(xs))]
            self.log(f'REEDS_SHEPP  len: {len(world_model.path)} ')
        elif not xs and world_model.path:
            # find closest point to vehicle
            pos = world_model.get_current_position()

            world_model.gps_path = [world_model.coords_transformer.get_global_coordinates_from_ipm_coords(p[0], p[1], pos) for p in world_model.path]
            gc = world_model.get_current_position()
            min_index = min(range(len(world_model.gps_path)), key=lambda index: calc_dist_point(gc, world_model.gps_path[index]))
            self.log(f'REEDS_SHEPP reduce: {min_index}')
            world_model.gps_path = world_model.gps_path[min_index:]
            world_model.path = world_model.path[min_index:]
            self.log(f'REEDS_SHEPP  len: {len(world_model.path)} ')
    
    def plan_path(self, world_model):
        if not world_model.goal_point:
            return world_model

        #world_model.path = None
        #self.plan_rrt_star_reeds_shepp(world_model)
        self.plan_reeds_shepp(world_model)

        #super().log(f'PATH: {world_model.path}')
        return world_model

    def on_event(self, event, scene=None):
        print("Emergency State")
        return None

    #@timeit
    def on_data(self, world_model):
        try:
            world_model = self.plan_path(world_model)
        except  Exception as err:
            super().error(''.join(traceback.TracebackException.from_exception(err).format()))

        return world_model
