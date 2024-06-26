class AbstractWorker:
    def __init__(self, node=None) -> None:
        self.node = node

    def log(self, message):
        self.node._logger.info(message)
    
    def logi(self, message):
        self.log('\033[92m' + message + '\033[0m')
    
    def logw(self, message):
        self.log('\033[93m' + message + '\033[0m')
    
    def loge(self, message):
        self.log('\033[91m' + message + '\033[0m')

    def error(self, message):
        self.node._logger.error(message)

    def on_event(self, event, scene=None):
        raise NotImplementedError

    def on_data(self, world_model):
        raise NotImplementedError
    