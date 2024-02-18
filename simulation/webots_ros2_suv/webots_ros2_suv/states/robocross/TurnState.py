from webots_ros2_suv.states.AbstractState import AbstractState

class TurnState(AbstractState):
    # Реализация методов для TurnState
    def __init__(self, *args, **kwargs) -> None:
        super().__init__( *args, **kwargs)
            
    def on_event(self, event, world_model=None):
        world_model.path = None
        return None

