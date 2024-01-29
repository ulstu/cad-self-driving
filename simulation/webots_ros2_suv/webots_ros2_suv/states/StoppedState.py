from AbstractState import AbstractState

class StoppedState(AbstractState):
    # Реализация методов для StartState
    def on_event(self, event, scene=None):
        print("Stopped State")
        return None

    def spin(self, scene_model):
        return None
