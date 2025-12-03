if __name__ == "__main__":
    import sys

    sys.path.insert(0, "../../..")

from src.templates.workerProcess import WorkerProcess
from src.utils.messages.allMessages import SpeedMotor, SteerMotor


class processLaneKeeper(WorkerProcess):
    """
    Acesta este procesul nostru de decizie.
    El va analiza datele si va trimite comenzi de miscare.
    """

    def __init__(self, queueList, logging, debugging=False):
        self.queueList = queueList
        self.logging = logging
        self.debugging = debugging
        super(processLaneKeeper, self).__init__(self.queueList)

    def run(self):
        super(processLaneKeeper, self).run()

    def _init_threads(self):
        """Initializam thread-ul de lucru."""
        if self.debugging:
            self.logging.info("LaneKeeper Process Initialized")
        return [self._work]

    def _work(self):
        """
        Aceasta functie ruleaza in bucla continua.
        Aici vom scrie logica de conducere autonoma.
        """
        import time

        # Asteptam putin la inceput ca sa se initializeze tot sistemul
        time.sleep(3)
        self.logging.info("LaneKeeper: STARTING MOVE SEQUENCE")

        # --- PASUL 1: Mergi Inainte ---
        # Cream mesajul de viteza
        # Action '1' inseamna setarea vitezei/directiei
        speed_command = {
            "action": "1",
            "speed": 0.2  # Viteza mica (m/s)
        }

        # Trimitem mesajul in coada "General" (Gateway il va prelua)
        # Nota: In arhitectura Brain, mesajele sunt obiecte, dar SerialHandler asteapta dictionare sau string-uri specifice.
        # Vom folosi o abordare simpla compatibila cu handler-ul lor.

        self.send_queue.put(speed_command)
        self.logging.info("LaneKeeper: Sending Speed 0.2")

        # Mergem 3 secunde
        time.sleep(3.0)

        # --- PASUL 2: Opreste ---
        stop_command = {
            "action": "1",
            "speed": 0.0
        }
        self.send_queue.put(stop_command)
        self.logging.info("LaneKeeper: STOPPING")

        # Dupa ce am terminat secventa, intram intr-o bucla de asteptare ca sa nu se termine procesul
        while True:
            time.sleep(1)