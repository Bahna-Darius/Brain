# Copyright (c) 2026, NovaVision
# All rights reserved.

from src.templates.workerprocess import WorkerProcess
from src.algorithms.TrafficLight.threads.threadTrafficLight import threadTrafficLight


class processTrafficLight(WorkerProcess):
    """Process running the traffic light detection thread."""

    def __init__(self, queueList, logging, ready_event, debugging: bool = False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        super(processTrafficLight, self).__init__(self.queuesList, ready_event)

    def _init_threads(self):
        tl_thread = threadTrafficLight(self.queuesList, self.logging, self.debugging)
        self.threads.append(tl_thread)
