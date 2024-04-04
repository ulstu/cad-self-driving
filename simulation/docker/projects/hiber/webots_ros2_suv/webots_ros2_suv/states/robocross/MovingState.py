from webots_ros2_suv.states.AbstractState import AbstractState
from webots_ros2_suv.lib.map_utils import is_point_in_polygon, calc_dist_point
import math

class MovingState(AbstractState):
    # Реализация методов для StartState

    def __init__(self, *args, **kwargs) -> None:
        super().__init__( *args, **kwargs)
        self.__cur_path_point = 1

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

    def find_next_goal_point(self, world_model):
        #self.log(f"{world_model.get_coord_corrections()}")
        if not world_model.global_map:
            return (0, 0)
        points_seg = [e['coordinates'] for e in world_model.global_map if e['name'] == 'moving']
        self.log(f'PATH SEGMENT: {world_model.cur_path_segment} {len(points_seg)}')
        points = points_seg[world_model.cur_path_segment]
        #self.log(f"POINTS: {points}")

        dists = []
        for p in points:
            dists.append(calc_dist_point(p, world_model.get_current_position()))

        if self.__cur_path_point < len(points) - 2:
            dist = calc_dist_point(points[self.__cur_path_point], world_model.get_current_position())
            self.log(f"DIST: {dist}")
            if dist < 2:
                self.__cur_path_point = self.__cur_path_point + 1
        else:
            self.__cur_path_point = len(points) - 1

        x, y = world_model.coords_transformer.get_relative_coordinates(points[self.__cur_path_point][0], 
                                                                       points[self.__cur_path_point][1], 
                                                                       pos=world_model.get_current_position(),
                                                                       pov_point=world_model.pov_point)
        #self.log(f"CURRENT POS: {world_model.get_current_position()}")
        # x = int(world_model.pov_point[0] - x) if world_model.pov_point[0] - x >=0 else 0
        # y = int(world_model.pov_point[1] - y) if world_model.pov_point[1] - y >=0 else 0
        self.log(f'CUR_POINT: {self.__cur_path_point} GOAL POINT: {x, y} X: {points[self.__cur_path_point][0]} Y:{points[self.__cur_path_point][1]} DISTS: {dists}')
        return (x, y)

    def on_event(self, event, world_model=None):
        lat, lon, o = world_model.get_current_position()
        #world_model.goal_point = (self.find_goal_point_x(world_model.ipm_image[10,:]), 10)
        world_model.goal_point = self.find_next_goal_point(world_model)

        event = None
        for p in world_model.global_map:
            if p['name'] == 'turn':
                if is_point_in_polygon(lat, lon, p['coordinates'][0]) and self.__cur_path_point > 2:
                    world_model.cur_turn_polygon = p['coordinates'][0]
                    self.__cur_path_point = 0
                    event = "turn"
            if p['name'] == 'stop':
                if is_point_in_polygon(lat, lon, p['coordinates'][0]):
                    self.__cur_path_point = 0
                    event = "stop"
        if event:
            world_model.path = None
        world_model.set_speed(30)
        self.drive(world_model, speed=25)
        return event

