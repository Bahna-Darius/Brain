# Copyright (c) 2026, NovaVision
# All rights reserved.

import threading

from src.templates.workerprocess import WorkerProcess
from src.control.ControlUnit.threads.threadControlUnit import threadControlUnit


class processControlUnit(WorkerProcess):
    """
    NovaVision 26.01.2026
    Process that runs the ControlUnit thread.
    ControlUnit reads PerceptionContext + StateChange and outputs DesiredSpeed/DesiredSteer.
    """

    def __init__(self, queueList, logger, ready_event=None, debugging=False):
        # IMPORTANT: WorkerProcess accepts only (queueList, logger, debugging)
        super(processControlUnit, self).__init__(queueList, logger, debugging)

        self.queueList = queueList
        self.logger = logger
        self.debugging = debugging

        # NovaVision: local ready event (do NOT pass into WorkerProcess)
        self.ready_event = ready_event

        self._stop_event = threading.Event()

    def _init_threads(self):
        th = threadControlUnit(self.queueList, self._stop_event, self.logger, self.debugging)
        self.threads.append(th)

        # NovaVision DEBUG: signal that ControlUnit is ready
        if self.ready_event is not None:
            self.ready_event.set()

        if self.debugging:
            print("[ControlUnit] thread started")

    def stop(self):
        self._stop_event.set()
        super(processControlUnit, self).stop()
