from AbstractWorker import AbstractWorker
import cv2
import numpy as np
import traceback
from webots_ros2_suv.lib.rrt_star import rrt_star, RRTTreeNode, visualize_path
from fastseg.image import colorize, blend


class PathPlanningWorker(AbstractWorker):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__( *args, **kwargs)

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


    def plan_path(self, world_model):
        world_model.goal_point = (self.find_goal_point_x(world_model.ipm_image[10,:]), 10)
        start_node = RRTTreeNode(world_model.pov_point[0], world_model.pov_point[1])
        goal_node = RRTTreeNode(world_model.goal_point[0],world_model.goal_point[1])
        
        max_iterations = 1000
        step_size = 20
        car_length = 30  # Длина автомобиля
        car_width = 30   # Ширина автомобиля
        
        world_model.path, nodes = rrt_star(start_node, goal_node, world_model.ipm_image, max_iterations, step_size, car_length, car_width)
        
        if world_model.path:
            super().log(f"Путь найден: {world_model.path}")
        else:
            self._logger.info("Путь не найден.")

        return world_model

    def draw_scene(self, world_model):
        colorized = world_model.ipm_colorized
        prev_point = None
        for n in world_model.path:
            if prev_point:
                cv2.line(colorized, prev_point, n, (0, 255, 255), 2)
            prev_point = n
        cv2.circle(colorized, world_model.pov_point, 9, (0, 255, 0), 5)
        cv2.circle(colorized, world_model.goal_point, 9, (255, 0, 0), 5)
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

    def on_data(self, world_model):
        try:
            super().log(f"PathPlanningWorker {str(world_model)}")
            world_model = self.plan_path(world_model)
            self.draw_scene(world_model)
        except  Exception as err:
            super().error(''.join(traceback.TracebackException.from_exception(err).format()))

        return world_model
