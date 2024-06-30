from webots_ros2_suv.states.AbstractState import AbstractState
from webots_ros2_suv.states.robocross.GPSFollowState import intersect_person_with_crosswalk

class StoppedState(AbstractState):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__( *args, **kwargs)

    def on_event(self, event, world_model=None):
        self.drive(world_model, speed=0.0)

        if world_model.traffic_light_state == 'green':
            if not world_model.pedestrian_on_crosswalk:
                return 'start_move'
        
        return None
