from AbstractState import AbstractState

class StartState(AbstractState):
    # Реализация методов для StartState
    def on_event(self, event, scene=None):
        print("Start State")
        return None

    def spin(self, scene_model):
        return None
