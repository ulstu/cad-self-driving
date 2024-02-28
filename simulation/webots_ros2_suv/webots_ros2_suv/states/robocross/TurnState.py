from webots_ros2_suv.states.AbstractState import AbstractState
from webots_ros2_suv.lib.map_utils import is_point_in_polygon, calc_dist_point
import math

class TurnState(AbstractState):
    # Реализация методов для TurnState
    def __init__(self, *args, **kwargs) -> None:
        super().__init__( *args, **kwargs)

    def find_next_goal_point(self, world_model):
        """
        Возвращает целевую точку для разворота.
        Точка определяется как первая точка того сегмента пути, который начинается в теукщей зоне разворота
        """
        points = [e['coordinates'] for e in world_model.global_map if e['name'] == 'moving']
        for p in points:
            if is_point_in_polygon(p[0][0], p[0][1], world_model.cur_turn_polygon):
                self.log(f'TURN GOAL POINT: {p[0][0], p[0][1]}')
                return (p[0][0], p[0][1])
        return None
            
    def on_event(self, event, world_model=None):
        lat, lon, o = world_model.get_current_position()
        abs_goal = self.find_next_goal_point(world_model)
        world_model.goal_point = world_model.get_relative_coordinates(abs_goal[0], abs_goal[1])
        if calc_dist_point(world_model.get_current_position(), world_model.goal_point) < 0.1:
            world_model.cur_path_segment = world_model.cur_path_segment + 1
            self.log("EXIT TURN STATE")
            event =  "start_move"
        return event


