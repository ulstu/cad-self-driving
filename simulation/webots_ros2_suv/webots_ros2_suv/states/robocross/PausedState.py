from webots_ros2_suv.states.AbstractState import AbstractState


class PausedState(AbstractState):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__( *args, **kwargs)
        self.count = -1

    def on_event(self, event, world_model=None):
        self.count += 1

        self.drive(world_model, speed=0.0)

        if self.count > 2:
            world_model.is_pause = False

        if not world_model.is_pause:
            return 'start_move'
        return None