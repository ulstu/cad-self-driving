from webots_ros2_suv.states.AbstractState import AbstractState

class StartState(AbstractState):
    # Реализация методов для StartState

    def __init__(self, *args, **kwargs) -> None:
        super().__init__( *args, **kwargs)
            
    def on_event(self, event, scene=None):
        super().log("Start State")
        return None

