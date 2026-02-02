# Copyright (c) 2026,   NovaVision 
# All rights reserved.

from src.templates.workerprocess import WorkerProcess
from src.algorithms.Perception.threads.threadPerception import threadPerception

class processPerception(WorkerProcess):
    def __init__(self, queueList, logging, ready_event=None, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        super(processPerception, self).__init__(self.queuesList, ready_event)

    def _init_threads(self):
        th = threadPerception(self.queuesList, self.logging, self.debugging)
        self.threads.append(th)

