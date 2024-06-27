from webots_ros2_suv.states.AbstractState import AbstractState


class PausedState(AbstractState):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__( *args, **kwargs)
        self._path_segment_changed = False

    def on_event(self, event, world_model=None):
        # Текущий сегмент пути, заданный в редакторе карт, изменяется 
        # только при заезде в зону терминала (world_model.is_pause == 1)?
        if not self._path_segment_changed and world_model.is_pause == 1:
            self._path_segment_changed = True
            world_model.cur_path_segment += 1

        self.drive(world_model, speed=0.0)

        if world_model.is_pause == 0:
            return "start_move"
        return None
