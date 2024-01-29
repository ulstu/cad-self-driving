class AbstractState:
    def on_event(self, event, scene=None):
        raise NotImplementedError

    def spin(self, scene_model):
        raise NotImplementedError
    
    