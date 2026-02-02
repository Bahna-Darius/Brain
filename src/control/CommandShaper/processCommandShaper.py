import multiprocessing
import threading

from src.control.CommandShaper.threads.threadCommandShaper import threadCommandShaper


class processCommandShaper(multiprocessing.Process):
    def __init__(self, queueList, logger, debugging=False):
        super().__init__()
        self.queueList = queueList
        self.logger = logger
        self.debugging = bool(debugging)   # ✅ NovaVision: enable prints
        self.stop_event = threading.Event()

    def run(self):
        shaper_thread = threadCommandShaper(
            self.queueList,
            self.stop_event,
            debugging=self.debugging,      # ✅ NovaVision
        )
        shaper_thread.start()
        shaper_thread.join()

    def stop(self):
        self.stop_event.set()
