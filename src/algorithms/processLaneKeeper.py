# from src.templates.workerprocess import WorkerProcess
# from src.utils.messages.allMessages import SpeedMotor, SteerMotor, Klem, StateChange
# from src.utils.messages.messageHandlerSender import messageHandlerSender
# from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
#
# import threading
# import platform
# import time
# import json
# import sys
# import os
#
#
# current_dir = os.path.dirname(os.path.abspath(__file__))
# brain_root = os.path.abspath(os.path.join(current_dir, '../../..'))
#
# if brain_root not in sys.path:
#     sys.path.append(brain_root)
#
#
# IS_SIMULATOR = platform.machine() == 'x86_64'
#
# if IS_SIMULATOR:
#     from rclpy.node import Node
#     from std_msgs.msg import String
#     import rclpy
#
#
# class ProcessLaneKeeper(WorkerProcess):
#     def __init__(self, queueList, logging, debugging=False):
#         self.queueList = queueList
#         self.logging = logging
#         self.debugging = debugging
#         super(ProcessLaneKeeper, self).__init__(self.queueList)
#
#         self.send_queue = self.queueList["General"]
#         self.ros_publisher = None
#         self.ros_node = None
#
#         # Sender-e oficiale pentru comenzi către SerialHandler
#         self.speed_sender = messageHandlerSender(self.queueList, SpeedMotor)
#         self.steer_sender = messageHandlerSender(self.queueList, SteerMotor)
#         self.klem_sender  = messageHandlerSender(self.queueList, Klem)
#
#         # Subscriber pentru schimbarea modului (DEFAULT / AUTO / MANUAL / STOP)
#         self.state_sub = messageHandlerSubscriber(self.queueList, StateChange, "FIFO", True)
#
#
#     # ------------------ ROS (simulator) ------------------ #
#     def _init_ros_node(self):
#         try:
#             if not rclpy.ok():
#                 rclpy.init()
#         except:
#             pass
#
#         self.ros_node = Node('brain_lane_keeper')
#         self.ros_publisher = self.ros_node.create_publisher(String, '/automobile/command', 10)
#         self.logging.info("✅ [LaneKeeper] ROS Connected!")
#         rclpy.spin(self.ros_node)
#
#
#     # ------------------ WorkerProcess hooks ------------------ #
#     def run(self):
#         super(ProcessLaneKeeper, self).run()
#
#
#     def _init_threads(self):
#         if self.debugging:
#             self.logging.info("ℹ️ [LaneKeeper] Process Initialized")
#
#         if IS_SIMULATOR:
#             ros_th = threading.Thread(target=self._init_ros_node, daemon=True)
#             ros_th.start()
#
#         work_th = threading.Thread(target=self._work, daemon=True)
#         work_th.start()
#
#         return [work_th]
#
#
#     # ------------------ Helper: trimite o singură comandă ------------------ #
#     def _send_command_once(self, speed, steer):
#         """Sends commands to both Simulator and Real Car"""
#         if IS_SIMULATOR and self.ros_publisher is not None:
#             ...
#         elif not IS_SIMULATOR:
#             # Mașină reală (SerialHandler) – TRIMITEM STRING-URI, nu float!
#             # SpeedMotor.msgType = "str", SteerMotor.msgType = "str"
#             speed_str = str(int(speed))   # ex: 250.0 -> "250"
#             steer_str = str(int(steer))   # ex: -10.0 -> "-10"
#
#             self.speed_sender.send(speed_str)
#             self.steer_sender.send(steer_str)
#
#
#     # ------------------ Helper: rulează o secvență X secunde ------------------ #
#     def run_sequence(self, duration, speed, steer, message):
#         """Trimite comanda continuu timp de 'duration' secunde"""
#         self.logging.info(f"👉 {message} (Time: {duration}s, Spd: {speed}, Str: {steer})")
#
#         t_end = time.time() + float(duration)
#         while time.time() < t_end:
#             self._send_command_once(speed, steer)
#             time.sleep(0.1)  # 10Hz
#
#
#     # ------------------ Helper: așteaptă AUTO de la state machine ------------------ #
#     def _wait_for_auto_mode(self):
#         """
#         Așteaptă până când mode-ul sistemului devine AUTO.
#         Citim mesaje de tip StateChange și căutăm 'auto' în conținut.
#         """
#         self.logging.info("⌛ [LaneKeeper] Aștept mod AUTO din dashboard/state machine...")
#
#         current_mode = None
#
#         while True:
#             msg = self.state_sub.receive()
#
#             if msg is None:
#                 # Nu a venit încă niciun mesaj de schimbare de stare
#                 time.sleep(0.1)
#                 continue
#
#             # Logăm ca să vezi EXACT ce vine pe StateChange
#             self.logging.info(f"🔄 [LaneKeeper] StateChange primit: {repr(msg)}")
#
#             # Depinde cum e definit msgType în allMessages.py:
#             #  - poate fi dict: {"mode": "auto", ...}
#             #  - sau string simplu: "auto"
#             mode = None
#             if isinstance(msg, dict):
#                 # încearcă să extragi cheia 'mode'
#                 mode = str(msg.get("mode", "")).lower()
#             else:
#                 # orice altceva convertim la string
#                 mode = str(msg).lower()
#
#             current_mode = mode
#
#             if "auto" in mode:
#                 self.logging.info("✅ [LaneKeeper] Mod AUTO detectat! Pornesc secvența.")
#                 break
#
#         return current_mode
#
#
#     # ------------------ MAIN LOGIC ------------------ #
#     def _work(self):
#         """Main logic – pornește DOAR când sistemul intră în AUTO din dashboard"""
#
#         # Dacă e simulator, așteptăm ROS
#         if IS_SIMULATOR:
#             self.logging.info("⏳ [LaneKeeper] Waiting for ROS...")
#             while self.ros_publisher is None:
#                 time.sleep(1)
#             time.sleep(2)
#
#         # 1) AȘTEPTĂM ca dashboard/state machine să pună sistemul în AUTO
#         self._wait_for_auto_mode()
#
#         # 2) DOAR PE MAȘINA REALĂ:
#         if not IS_SIMULATOR:
#             # Așteptăm ca serverul să dea start (cap robot ~15s)
#             self.logging.info("⏳ [LaneKeeper] Aștept 15s (cap robot / start oficial)")
#             time.sleep(15.0)
#
#             # Pornim motorul logic: Klem = 30
#             self.logging.info("⚙️ [LaneKeeper] Trimit Klem=30 (motor ON)")
#             self.klem_sender.send("30")
#             time.sleep(0.5)  # mică pauză pentru procesare
#
#         # 3) RULĂM TESTUL EFECTIV
#         self.logging.info("🚀 [LaneKeeper] PORNESC TESTUL CU VITEZĂ REALĂ")
#
#         # 3.1) MERGE ÎNAINTE DREPT (puțin mai scurt)
#         self.run_sequence(
#             duration=2.0,
#             speed=200.0,
#             steer=0.0,
#             message="TEST: inainte drept 2s la 200 cm/s"
#         )
#
#         # Mică pauză între comenzi (ca să nu pară “lipite”)
#         time.sleep(0.5)
#
#         # 3.2) VIRAJ PUTERNIC (±25 pentru efect vizibil)
#         self.run_sequence(
#             duration=3.0,
#             speed=150.0,      # puțin mai încet ca să fie controlabil
#             steer=-25.0,      # unghi maxim de viraj
#             message="TEST: viraj 3s la 150 cm/s, steer=-25"
#         )
#
#
#         # 4) STOP
#         self.logging.info("🛑 [LaneKeeper] Test gata, opresc masina")
#         self._send_command_once(0.0, 0.0)
#
#         # 5) Ținem procesul în viață (dar fără altă logică deocamdată)
#         while True:
#             time.sleep(1)
