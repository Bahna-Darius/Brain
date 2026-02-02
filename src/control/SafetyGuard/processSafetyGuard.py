import multiprocessing
import threading

from src.control.SafetyGuard.threads.threadSafetyGuard import threadSafetyGuard


class processSafetyGuard(multiprocessing.Process):
    def __init__(self, queueList, logger, debugging=False):
        super().__init__()
        self.queueList = queueList
        self.logger = logger
        self.debugging = bool(debugging)  # ✅ NovaVision Debugger flag
        self.stop_event = threading.Event()

    def run(self):
        guard_thread = threadSafetyGuard(
            self.queueList,
            self.stop_event,
            debugging=self.debugging,      # ✅ NovaVision
        )
        guard_thread.start()
        guard_thread.join()

    def stop(self):
        self.stop_event.set()
