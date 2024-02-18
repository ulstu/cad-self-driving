class AbstractState:

    def __init__(self, node=None) -> None:
        self.node = node

    def on_event(self, event, scene=None):
        raise NotImplementedError

    def log(self, message):
        self.node._logger.info(message)    