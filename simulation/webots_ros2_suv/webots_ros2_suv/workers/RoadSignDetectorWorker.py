from AbstractWorker import AbstractWorker

class RoadSignDetectorWorker(AbstractWorker):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__( *args, **kwargs)

    def on_event(self, event, scene=None):
        print("Emergency State")
        return None

    def on_data(self, world_model):
        #super().log("RoadSignDetectorWorker data received")
        return world_model
