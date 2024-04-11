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
                return (p[0][0], p[0][1])
        return None
            
    def on_event(self, event, world_model=None):
        abs_goal = self.find_next_goal_point(world_model)
        pos = world_model.get_current_position()
        world_model.goal_point = world_model.coords_transformer.get_relative_coordinates(
            abs_goal[0], 
            abs_goal[1],
            pos=pos,
            pov_point=world_model.pov_point)
        target_dist = calc_dist_point(world_model.get_current_position(), abs_goal)
        self.log(f'TARGET DIST: {target_dist} TURN GOALS: [{abs_goal}] [{world_model.goal_point}]')
        if target_dist < 0.5:
            world_model.cur_path_segment = world_model.cur_path_segment + 1
            self.log("EXIT TURN STATE")
            event = "start_move"
        point_count = len(world_model.path) - 2 if world_model.path else 0
        self.drive(world_model, angle_points_count=point_count, speed=5)
        return event


