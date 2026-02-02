#
# import time
#
# from src.templates.workerprocess import WorkerProcess
# from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
# from src.utils.messages.messageHandlerSender import messageHandlerSender
# from src.utils.messages.allMessages import StateChange
# import src.utils.messages.allMessages as allMessages
#
#
# class process3PointsTurn(WorkerProcess):
#
#     AUTO_STATE_NAME = "AUTO"
#
#     SPEED_MESSAGE_ENUM = allMessages.SpeedMotor
#     STEER_MESSAGE_ENUM = allMessages.SteerMotor
#
#     # ===== CONFIG =====
#     SPEED_STRAIGHT = "150"     # straight segments
#     SPEED_TURN_FWD = "110"     # slower helps rotation with small steering angle
#     SPEED_TURN_REV = "-120"
#
#     STEER_LEFT  = "-250"
#     STEER_RIGHT = "250"
#     STEER_STRAIGHT = "0"
#
#     FORWARD_TIME_1 = 6.0
#
#     # These two are your main tuning knobs:
#     REV_TURN_TIME = 5.0        # increase to rotate more in reverse
#     FWD_TURN_TIME = 3.5        # increase to rotate more in forward
#
#     # optional small settle at end:
#     FINAL_CREEP_TIME = 0.8
#
#     PAUSE_TIME   = 1.0
#     SEND_PERIOD  = 0.8         # slightly faster update than 1s
#     # ==================
#
#     def __init__(self, queueList, logging, ready_event=None, debugging=False):
#         self.queueList = queueList
#         self.logger = logging
#         self.debugging = debugging
#         self.ready_event = ready_event
#         self.running = True
#
#         self.state_change_sub = messageHandlerSubscriber(
#             self.queueList, StateChange, "lastOnly", True
#         )
#
#         self.speed_sender = messageHandlerSender(self.queueList, self.SPEED_MESSAGE_ENUM)
#         self.steer_sender = messageHandlerSender(self.queueList, self.STEER_MESSAGE_ENUM)
#
#         super(process3PointsTurn, self).__init__(self.queueList, self.ready_event)
#
#     # ==========================================
#
#     def stop(self):
#         self.running = False
#         super(process3PointsTurn, self).stop()
#
#     def run(self):
#         if self.ready_event:
#             self.ready_event.set()
#
#         self.logger.info("[3PointsTurn] process started")
#
#         while self.running:
#             message = self.state_change_sub.receive()
#
#             if message == self.AUTO_STATE_NAME:
#                 self.logger.info("[3PointsTurn] AUTO detected → 3-point turn script")
#                 self._run_sequence()
#                 self.logger.info("[3PointsTurn] Script finished")
#
#             time.sleep(0.05)
#
#     # ==========================================
#
#     def _run_sequence(self):
#
#         # 1) Forward 10s
#         self._run_phase(self.SPEED_STRAIGHT, self.STEER_STRAIGHT, self.FORWARD_TIME_1)
#         self._stop(self.PAUSE_TIME)
#
#         # 2) Reverse with full lock LEFT
#         self._run_phase(self.SPEED_TURN_FWD, self.STEER_LEFT, self.FWD_TURN_TIME)
#         self._stop(self.PAUSE_TIME)
#
#         # 3) Forward with full lock RIGHT
#         self._run_phase(self.SPEED_TURN_REV, self.STEER_RIGHT, self.REV_TURN_TIME)
#         self._stop(self.PAUSE_TIME)
#
#         # 4) Reverse straight to line up (short)
#         self._run_phase(self.SPEED_TURN_FWD, self.STEER_LEFT, 5.75)
#         self._stop(self.PAUSE_TIME)
#
#         # 5) Forward again 10s
#         self._run_phase(self.SPEED_STRAIGHT, self.STEER_STRAIGHT, 4.0)
#
#         # final stop
#         self._send_speed("0"); time.sleep(0.1); self._send_steer("0")
#
#
#     # ==========================================
#
#     def _run_phase(self, speed, steer, duration):
#         start = time.time()
#         while self.running and (time.time() - start) < duration:
#             self._send_speed(speed)
#             time.sleep(0.1)
#             self._send_steer(steer)
#             time.sleep(self.SEND_PERIOD)
#
#     def _stop(self, duration):
#         self._send_speed("0")
#         time.sleep(0.1)
#         self._send_steer("0")
#         time.sleep(duration)
#
#     # ==========================================
#
#     def _send_speed(self, value):
#         if self.debugging:
#             self.logger.info(f"[AutoForward] speed={value}")
#         self.speed_sender.send(value)
#
#     def _send_steer(self, value):
#         if self.debugging:
#             self.logger.info(f"[AutoForward] steer={value}")
#         self.steer_sender.send(value)