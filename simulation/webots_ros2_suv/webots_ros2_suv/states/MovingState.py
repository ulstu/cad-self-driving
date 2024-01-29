from AbstractState import AbstractState

class MovingState(AbstractState):
    # Реализация методов для StartState
    def on_event(self, event, scene=None):
        print("Moving State")
        return None

    def spin(self, scene_model):
        return None
