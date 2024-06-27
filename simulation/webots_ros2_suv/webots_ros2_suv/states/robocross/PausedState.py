from webots_ros2_suv.states.AbstractState import AbstractState


class PausedState(AbstractState):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__( *args, **kwargs)

    def on_event(self, event, world_model=None):
        self.drive(world_model, speed=0.0)

        if not world_model.is_pause:
            return 'start_move'
        return None
