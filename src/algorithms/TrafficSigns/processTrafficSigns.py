from src.templates.workerprocess import WorkerProcess
from src.algorithms.TrafficSigns.threads.threadTrafficSigns import threadTrafficSigns


class processTrafficSigns(WorkerProcess):
    def __init__(self, queueList, logging, ready_event=None, debugging=False):
        self.queueList = queueList
        self.logging = logging
        self.debugging = debugging
        super(processTrafficSigns, self).__init__(queueList, ready_event)

    def _init_threads(self):
        tsdTh = threadTrafficSigns(self.queueList, self.logging, self.debugging)
        self.threads.append(tsdTh)
        