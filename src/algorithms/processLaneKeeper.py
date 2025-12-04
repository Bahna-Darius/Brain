from src.templates.workerprocess import WorkerProcess
from src.utils.messages.allMessages import SpeedMotor, SteerMotor
import threading
import platform
import time
import json
import sys
import os



current_dir = os.path.dirname(os.path.abspath(__file__))
brain_root = os.path.abspath(os.path.join(current_dir, '../../..'))

if brain_root not in sys.path:
    sys.path.append(brain_root)


IS_SIMULATOR = platform.machine() == 'x86_64'

if IS_SIMULATOR:
    from rclpy.node import Node
    from std_msgs.msg import String
    import rclpy


class ProcessLaneKeeper(WorkerProcess):
    def __init__(self, queueList, logging, debugging=False):
        self.queueList = queueList
        self.logging = logging
        self.debugging = debugging
        super(ProcessLaneKeeper, self).__init__(self.queueList)

        self.send_queue = self.queueList["General"]
        self.ros_publisher = None
        self.ros_node = None

    def _init_ros_node(self):
        try:
            if not rclpy.ok():
                rclpy.init()
        except:
            pass

        self.ros_node = Node('brain_lane_keeper')
        self.ros_publisher = self.ros_node.create_publisher(String, '/automobile/command', 10)
        self.logging.info("✅ [LaneKeeper] ROS Connected!")
        rclpy.spin(self.ros_node)

    def run(self):
        super(ProcessLaneKeeper, self).run()


    def _init_threads(self):
        if self.debugging:
            self.logging.info("ℹ️ [LaneKeeper] Process Initialized")

        if IS_SIMULATOR:
            ros_th = threading.Thread(target=self._init_ros_node, daemon=True)
            ros_th.start()

        work_th = threading.Thread(target=self._work)
        work_th.daemon = True
        work_th.start()

        return [work_th]


    def _send_command_once(self, speed, steer):
        """Sends commands to both Simulator and Real Car"""
        if IS_SIMULATOR and self.ros_publisher is not None:
            # 1. Full JSON (New Standard)
            msg_full = {"action": "1", "speed": float(speed), "steerAngle": float(steer)}
            self.ros_publisher.publish(String(data=json.dumps(msg_full)))

            # 2. Separate Messages (Old Standard/Serial Style) - Critical for some simulators
            cmd_speed = {"action": "1", "speed": float(speed)}
            self.ros_publisher.publish(String(data=json.dumps(cmd_speed)))

            cmd_steer = {"action": "2", "steerAngle": float(steer)}
            self.ros_publisher.publish(String(data=json.dumps(cmd_steer)))

        elif not IS_SIMULATOR:
            # Real Car (SerialHandler)
            cmd_speed = {"Owner": "LK", "msgID": SpeedMotor.msgID, "msgType": SpeedMotor.msgType,
                         "msgValue": float(speed)}
            self.send_queue.put(cmd_speed)

            cmd_steer = {"Owner": "LK", "msgID": SteerMotor.msgID, "msgType": SteerMotor.msgType,
                         "msgValue": float(steer)}
            self.send_queue.put(cmd_steer)


    def run_sequence(self, duration, speed, steer, message):
        """Sends the command continuously for 'duration' seconds"""
        self.logging.info(f"👉 {message} (Time: {duration}s, Spd: {speed}, Str: {steer})")

        t_end = time.time() + duration
        while time.time() < t_end:
            self._send_command_once(speed, steer)
            time.sleep(0.1)  # 10Hz

    def _work(self):
        """Main Logic"""
        if IS_SIMULATOR:
            self.logging.info("⏳ Waiting for ROS...")
            while self.ros_publisher is None:
                time.sleep(1)
            time.sleep(2)

        self.logging.info("🚀 STARTING REVERSE DEMO")

        # 1. FORWARD (Straight)
        self.run_sequence(3.0, 0.3, 0.0, "MOVING FORWARD")

        # 2. RIGHT TURN (Forward)
        self.run_sequence(4.0, 0.3, 25.0, "TURNING RIGHT")

        # 3. STOP & STRAIGHTEN WHEELS (Crucial Step!)
        # We stop the car and set steer to 0.0 before reversing
        self.run_sequence(1.0, 0.0, 0.0, "STOP & STRAIGHTEN WHEELS")

        # 4. REVERSE (Straight Backwards)
        # Negative speed (-0.3) and Zero Steer (0.0)
        self.run_sequence(5.0, -0.3, 0.0, "REVERSING STRAIGHT")

        # 5. FINAL STOP
        self.logging.info("🛑 MISSION COMPLETE")
        self._send_command_once(0.0, 0.0)

        while True:
            time.sleep(1)