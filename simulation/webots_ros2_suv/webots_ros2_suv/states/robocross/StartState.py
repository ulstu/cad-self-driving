from webots_ros2_suv.states.AbstractState import AbstractState
import time

class StartState(AbstractState):
    # Реализация методов для StartState

    def __init__(self, *args, **kwargs) -> None:
        super().__init__( *args, **kwargs)
        self.t1 = time.time()
            
    def on_event(self, event, world_model=None):
        if world_model:
            super().log(f"Start State. Traffic light state: {world_model.traffic_light_state}")
            if world_model.traffic_light_state == "green":
                return "start_move"
        return event

