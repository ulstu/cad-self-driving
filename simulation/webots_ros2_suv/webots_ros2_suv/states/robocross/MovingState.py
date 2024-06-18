from webots_ros2_suv.states.AbstractState import AbstractState
from webots_ros2_suv.lib.map_utils import is_point_in_polygon, calc_dist_point
import math

class MovingState(AbstractState):

    def __init__(self, *args, **kwargs) -> None:
        super().__init__( *args, **kwargs)
        self.runs = 0
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
    
    def is_obstacle_near(self, world_model, x, y, obstacle_val, robot_radius):
        for i in range(x - robot_radius, x + robot_radius):
            for j in range(y - robot_radius, y + robot_radius):
                if world_model.ipm_image[j, i] != obstacle_val:
                    return False
        return True


    def find_next_goal_point(self, world_model):
        if not world_model.global_map:
            return (0, 0)
        points = [e['coordinates'] for e in world_model.global_map if e['name'] == 'moving' and 'seg_num' in e and int(e['seg_num']) == world_model.cur_path_segment]
        self.log(f'PATH SEGMENT: {world_model.cur_path_segment} PATH: {points}')

        points = points[0]

        dists = []
        for p in points:
            dists.append(calc_dist_point(p, world_model.get_current_position()))
        self.__cur_path_point = world_model.cur_path_point
        if self.__cur_path_point < len(points) - 1:
            x, y = world_model.coords_transformer.get_relative_coordinates(points[self.__cur_path_point][0], points[self.__cur_path_point][1], world_model.get_current_position(), world_model.pov_point)
            dist = calc_dist_point(points[self.__cur_path_point], world_model.get_current_position())
            world_model.params.append({"point_dist": dist})
            if dist < self.config['change_point_dist'] or (world_model.ipm_image.shape[1] > x and world_model.ipm_image.shape[0] > y and not self.is_obstacle_near(world_model, x, y, 100, 8)):
                self.__cur_path_point = self.__cur_path_point + 1
            self.log(f"DIST {dist} CUR POINT: {self.__cur_path_point} SEG: {world_model.cur_path_segment}")
        else:
            self.__cur_path_point = len(points) - 1
        world_model.cur_path_point = self.__cur_path_point
        x, y = world_model.coords_transformer.get_relative_coordinates(points[self.__cur_path_point][0], 
                                                                       points[self.__cur_path_point][1], 
                                                                       pos=world_model.get_current_position(),
                                                                       pov_point=world_model.pov_point)
        return (x, y)

    def on_event(self, event, world_model=None):
        self.runs = self.runs + 1
        # if world_model.traffic_light_state == "red":
        #     return "stop"
        lat, lon, o = world_model.get_current_position()
        #world_model.goal_point = (self.find_goal_point_x(world_model.ipm_image[10,:]), 10)
        world_model.goal_point = self.find_next_goal_point(world_model)

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
        if not world_model.is_obstacle_before_path_point(filter_num=10, log=self.log):
            event = "start_gps_follow"
        if world_model.is_lane_road():
            event = "start_lane_follow"

        if event:
            world_model.path = None
        world_model.set_speed(speed)
        self.drive(world_model, speed=speed)
        return event

