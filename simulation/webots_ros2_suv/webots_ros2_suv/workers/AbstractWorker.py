class AbstractWorker:
    def __init__(self, node=None) -> None:
        self.node = node

    def log(self, message):
        self.node._logger.info(message)

    def error(self, message):
        self.node._logger.error(message)

    def on_event(self, event, scene=None):
        raise NotImplementedError

    def on_data(self, world_model):
        raise NotImplementedError
    