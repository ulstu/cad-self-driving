from webots_ros2_suv.states.AbstractState import AbstractState
import time

class StartState(AbstractState):
    # Реализация методов для StartState

    def __init__(self, *args, **kwargs) -> None:
        super().__init__( *args, **kwargs)
        self.t1 = time.time()
            
    def on_event(self, event, world_model=None):
        super().log("Start State")
        #self.drive(world_model, speed=0)
        if world_model:
            if world_model.traffic_light_state == "green":
                return "start_move"
        return None

