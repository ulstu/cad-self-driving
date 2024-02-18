from webots_ros2_suv.states.AbstractState import AbstractState
from webots_ros2_suv.lib.map_utils import is_point_in_polygon
import math

class MovingState(AbstractState):
    # Реализация методов для StartState
    __EARTH_RADIUS_KM = 6371.0  # Радиус Земли в километрах

    def __init__(self, *args, **kwargs) -> None:
        super().__init__( *args, **kwargs)
        self.__cur_path_point = 0

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

    def calc_dist_point(self, p1, p2):
        """
        Возвращает дистанцию между двумя точками, заданными кортежами (lat, lon),
        используя формулу гаверсинуса.
        """
        lat1, lon1 = p1[0], p1[1]#map(math.radians, p1)
        lat2, lon2 = p2[0], p2[1]#map(math.radians, p2)

        delta_lat = lat2 - lat1
        delta_lon = lon2 - lon1

        a = math.sin(delta_lat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(delta_lon / 2)**2
        c = 2 * math.asin(math.sqrt(a))

        distance_km = self.__EARTH_RADIUS_KM * c
        return distance_km
    
    def find_next_goal_point(self, world_model):
        self.log(f"{world_model.get_coord_corrections()}")
        if not world_model.global_map:
            return (0, 0)
        points = [e['coordinates'] for e in world_model.global_map if e['name'] == 'moving'][0]

        if len(points) > self.__cur_path_point - 2:
            if self.calc_dist_point(points[self.__cur_path_point + 1], world_model.get_current_position()) < self.calc_dist_point(points[self.__cur_path_point], world_model.get_current_position()):
                self.__cur_path_point = self.__cur_path_point + 2
        if len(points) < self.__cur_path_point + 1:
            self.__cur_path_point = len(points) - 1

        x, y = world_model.get_relative_coordinates(points[self.__cur_path_point][0], points[self.__cur_path_point][1])
        self.log(f"CURRENT POS: {world_model.get_current_position()}")
        # x = int(world_model.pov_point[0] - x) if world_model.pov_point[0] - x >=0 else 0
        # y = int(world_model.pov_point[1] - y) if world_model.pov_point[1] - y >=0 else 0
        self.log(f'GOAL POINT: {x, y} CUR_POINT: {self.__cur_path_point}')
        return (x, y)

    def on_event(self, event, world_model=None):
        lat, lon, o = world_model.get_current_position()
        #world_model.goal_point = (self.find_goal_point_x(world_model.ipm_image[10,:]), 10)
        world_model.goal_point = self.find_next_goal_point(world_model)

        event = None
        for p in world_model.global_map:
            if p['name'] == 'turn':
                if is_point_in_polygon(lat, lon, p['coordinates'][0]):
                    event = "turn"
            # if p['name'] == 'finish':
            #     if is_point_in_polygon(lat, lon, p['coordinates'][0]):
            #         event = "stop"
        if event:
            world_model.path = None
        return event

