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


class ReedsSheppPlpanningWorker(AbstractWorker):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__( *args, **kwargs)

    def plan_rrt_star_reeds_shepp(self, world_model):
        start = [world_model.pov_point[0], world_model.pov_point[1], np.deg2rad(0.0)]
        goal = [world_model.goal_point[0],world_model.goal_point[1], np.deg2rad(0.0)]
        max_iter = 200
        step_size = 10
        rand_area=[300.0, 500.0, 100, 200]
        rrt_star_reeds_shepp = RRTStarReedsShepp(start, goal,
                                                world_model.ipm_image,
                                                rand_area=rand_area, step_size=step_size, max_iter=max_iter)
        path = rrt_star_reeds_shepp.planning(animation=False)
        if path:
            world_model.path = [(int(p[0]), int(p[1])) for p in path]
    
    def plan_path(self, world_model):
        if not world_model.goal_point:
            return world_model

        world_model.path = None
        self.plan_rrt_star_reeds_shepp(world_model)

        super().log(f'PATH: {world_model.path}')
        return world_model

    def draw_scene(self, world_model):
        colorized = world_model.ipm_colorized
        prev_point = None
        if not world_model.path:
            return
        for n in world_model.path:
            if prev_point:
                cv2.line(colorized, prev_point, n, (0, 255, 255), 2)
            prev_point = n
        cv2.circle(colorized, world_model.pov_point, 9, (0, 255, 0), 5)
        cv2.circle(colorized, world_model.goal_point, 9, (255, 0, 0), 5)
        points = [e['coordinates'] for e in world_model.global_map if e['name'] == 'moving'][0]

        for p in points:
            x, y = world_model.get_relative_coordinates(p[0], p[1])
            cv2.circle(colorized, (x, y), 8, (0, 0, 255), 2)

        colorized = cv2.resize(colorized, (500, 500), cv2.INTER_AREA)
        cv2.imshow("colorized seg", colorized)


        #cv2.imshow("composited image", np.asarray(colorize(world_model.ipm_seg)))
        #img_tracks = draw_absolute_tracks(self.__track_history_bev, 500, 500, self._logger)
        #cv2.imshow("yolo drawing", img_tracks)


        if cv2.waitKey(10) & 0xFF == ord('q'):
            return

    def on_event(self, event, scene=None):
        print("Emergency State")
        return None

    #@timeit
    def on_data(self, world_model):
        try:
            # super().log(f"PathPlanningWorker {str(world_model)}")
            world_model = self.plan_path(world_model)
            self.draw_scene(world_model)
        except  Exception as err:
            super().error(''.join(traceback.TracebackException.from_exception(err).format()))

        return world_model
