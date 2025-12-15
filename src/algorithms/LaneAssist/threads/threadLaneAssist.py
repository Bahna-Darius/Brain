import time
import platform

from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (
    mainCamera,
    SpeedMotor,
    SteerMotor,
    StateChange,
)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender


IS_SIMULATOR = platform.machine() == "x86_64"


class threadLaneAssist(ThreadWithStop):
    """LaneAssist thread.

    - Listens to StateChange (DEFAULT / STOP / AUTO...)
    - When entering AUTO:
        * Does NOT send Klem anymore (you set it from the dashboard)
        * Does NOT wait 15s anymore
        * Runs a test sequence: forward + turn + stop
    """

    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging

        self.current_mode = "DEFAULT"
        self.has_run_test = False
        self.auto_start_time = None

        self.last_cmd_time = 0.0

        self.subscribe()

        super(threadLaneAssist, self).__init__()

    # ------------------ Messages (subscribe + senders) ------------------ #
    def subscribe(self):
        """Subscribes to the messages we are interested in."""
        # Mode changes (DEFAULT / STOP / AUTO / MANUAL)
        self.state_sub = messageHandlerSubscriber(
            self.queuesList, StateChange, "FIFO", True
        )

        # Commands to SerialHandler (via Gateway)
        self.speed_sender = messageHandlerSender(self.queuesList, SpeedMotor)
        self.steer_sender = messageHandlerSender(self.queuesList, SteerMotor)

        # In the future: we can also add the main image
        # self.camera_sub = messageHandlerSubscriber(self.queuesList, mainCamera, "lastOnly", True)

    # ------------------ State change handler ------------------ #
    def state_change_handler(self):
        """Updates the current mode based on StateChange messages."""
        msg = self.state_sub.receive()
        if msg is None:
            return

        self.logging.info(f"[LaneAssist] StateChange received in thread: {repr(msg)}")

        # msg can be 'AUTO', 'STOP', etc. or dict {'mode': 'AUTO', ...}
        if isinstance(msg, dict):
            mode = str(msg.get("mode", "")).upper()
        else:
            mode = str(msg).upper()

        self.current_mode = mode

        if mode != "AUTO":
            # we exit AUTO: reset the timer for the test sequence
            self.auto_start_time = None
            self.logging.info(f"[LaneAssist] Current mode: {self.current_mode}")

        # NOTE: we do not reset has_run_test here, to avoid rerunning the test
        # every time you enter/exit AUTO in the same session.

    # ------------------ Helper: send speed + steering commands ------------------ #
    def _send_speed_steer(self, speed, steer):
        """Sends speed and steering commands.

        SpeedMotor.msgType and SteerMotor.msgType are 'str',
        so we send strings.
        """
        if IS_SIMULATOR:
            # here we can integrate the simulator later
            return

        now = time.time()
        # throttling at ~10 Hz
        if now - self.last_cmd_time < 0.1:
            return
        self.last_cmd_time = now

        speed_str = str(int(speed))
        steer_str = str(int(steer))

        self.speed_sender.send(speed_str)
        self.steer_sender.send(steer_str)

    # ------------------ MAIN LOOP ------------------ #
    def thread_work(self):
        """Main LaneAssist loop.

        - Waits for AUTO
        - When entering AUTO and the test has not run:
            * immediately starts the test sequence:
              - 0–2s: straight forward (200 cm/s)
              - 2–5s: turn (150 cm/s, steer=-25)
              - >5s: stop
        """

        # 1) If we are not in AUTO or the test has already run, do nothing
        if self.current_mode != "AUTO":
            time.sleep(0.05)
            return

        if self.has_run_test:
            # test has already run; for now LaneAssist does nothing more
            time.sleep(0.05)
            return

        # 2) We just entered AUTO → start the timer for the test sequence
        if self.auto_start_time is None:
            self.auto_start_time = time.time()
            self.logging.info("[LaneAssist] Entered AUTO, starting test sequence immediately.")
            return

        # 3) How much time has passed since we started the sequence
        test_elapsed = time.time() - self.auto_start_time

        # 3.1) Phase 1: GO STRAIGHT FORWARD (0–2s)
        if 0.0 <= test_elapsed < 4.0:
            if self.debugging:
                self.logging.info("[LaneAssist] Phase 1: straight forward (200 cm/s)")
            self._send_speed_steer(200, 0)  # 200 cm/s, steering straight
            return

        # 3.2) Phase 2: SHARP TURN (2–5s)
        if 2.0 <= test_elapsed < 5.0:
            if self.debugging:
                self.logging.info("[LaneAssist] Phase 2: turn (150 cm/s, steer=-25)")
            self._send_speed_steer(150, -25)  # 150 cm/s, left turn
            return

        # 3.3) Phase 3: STOP (after 5s)
        if test_elapsed >= 5.0:
            self.logging.info("🛑 [LaneAssist] Test finished, stopping car")
            self._send_speed_steer(0, 0)
            self.has_run_test = True
            return