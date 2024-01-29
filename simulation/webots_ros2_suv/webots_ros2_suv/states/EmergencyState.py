from AbstractState import AbstractState

class EmergencyState(AbstractState):
    # Реализация методов для StartState
    def on_event(self, event, scene=None):
        print("Emergency State")
        return None

    def spin(self, scene_model):
        return None
