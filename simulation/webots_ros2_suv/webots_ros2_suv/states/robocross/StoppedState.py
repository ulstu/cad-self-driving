from webots_ros2_suv.states.AbstractState import AbstractState

class StoppedState(AbstractState):
    # Реализация методов для StartState
    def __init__(self, *args, **kwargs) -> None:
        super().__init__( *args, **kwargs)
            
    def on_event(self, event, world_model=None):
        print("Stopped State")
        self.drive(world_model, speed=0)
        if world_model.traffic_light_state == "green":
            return "start_move"
        return None

