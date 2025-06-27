from tramola.task import Task

class GoTo(Task):
    def __init__(self, vehicle, point):
        super().__init__(vehicle)
        
    def start(self):
        pass

    def detection_callback(self):
        pass