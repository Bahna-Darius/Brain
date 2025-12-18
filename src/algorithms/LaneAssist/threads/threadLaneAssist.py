from src.utils.messages.allMessages import (mainCamera, StateChange, SpeedMotor, SteerMotor, serialCamera, LaneKeeping)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.templates.threadwithstop import ThreadWithStop

import numpy as np
import platform
import time
import cv2
import base64

IS_SIMULATOR = platform.machine() == "x86_64"

# target speed (in cm/s) in AUTO mode – you can adjust it
TARGET_SPEED_CM_S = 180.0  # ~ 6.5 km/h


class threadLaneAssist(ThreadWithStop):
    """
    LaneAssist thread.
    Purpose:
    - Listens to StateChange (DEFAULT / STOP / AUTO ...)
    - When in AUTO:
        * reads frames from mainCamera
        * detects the lane and computes the lateral error
        * uses a fuzzy controller to compute steering
        * sends SpeedMotor + SteerMotor to the Nucleo
    """

    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging

        self.current_mode = "DEFAULT"
        self._last_no_cam_log = 0.0
        self.last_cmd_time = 0.0
        self.last_speed = None
        self.last_steer = None

        self.subscribe()

        # senders for commands to the car (must be str according to allMessages)
        self.speed_sender = messageHandlerSender(self.queuesList, SpeedMotor)
        self.steer_sender = messageHandlerSender(self.queuesList, SteerMotor)
        # self.debug_cam_sender = messageHandlerSender(self.queuesList, serialCamera)

        super(threadLaneAssist, self).__init__()

    # ------------------ Subscribers & Senders ------------------ #
    #
    # def subscribe(self):
    #     self.state_sub = messageHandlerSubscriber(self.queuesList, StateChange, "lastOnly", True)
    #     self.serialCameraSubscriber = messageHandlerSubscriber(self.queuesList, serialCamera, "lastOnly", True)
    #     self.mainCameraSubscriber = messageHandlerSubscriber(self.queuesList, mainCamera, "lastOnly", True)

    # ------------------ State change handler ------------------ #
    def state_change_handler(self):
        msg = self.state_sub.receive()
        if msg is None:
            return

        mode = msg["value"] if isinstance(msg, dict) and "value" in msg else msg
        if isinstance(mode, bytes):
            mode = mode.decode(errors="ignore")
        self.current_mode = str(mode)
        self.logging.info(f"[LaneAssist] StateChange received: '{self.current_mode}'")

    # ------------------ Helper: sending commands ------------------ #
    def _send_speed_steer(self, speed, steer):
        """
        Sends speed and steering commands.
        - speed: cm/s (float)
        - steer: degrees [-25, 25] (float)

        In messages we must send STRINGs.
        """
        if IS_SIMULATOR:
            # here we can add ROS later
            return

        now = time.time()

        # simple throttling at ~10 Hz
        if now - self.last_cmd_time < 0.1:
            return
        self.last_cmd_time = now

        speed_str = str(int(speed))
        steer_str = str(int(steer))

        if self.debugging:
            self.logging.info(f"[LaneAssist] CMD speed={speed_str} steer={steer_str}")

        self.speed_sender.send(speed_str)
        self.steer_sender.send(steer_str)

    # ------------------ Helper: frame decoding ------------------ #
    def _decode_frame(self, msg):
        try:
            if msg is None:
                return None

            # for you, msg is a base64 STRING
            if isinstance(msg, str):
                raw = base64.b64decode(msg)
                img_array = np.frombuffer(raw, dtype=np.uint8)
                frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                return frame

            # fallbacks, in case something changes in the future
            if isinstance(msg, (bytes, bytearray)):
                img_array = np.frombuffer(msg, dtype=np.uint8)
                return cv2.imdecode(img_array, cv2.IMREAD_COLOR)

            if isinstance(msg, dict) and "msgValue" in msg:
                raw = base64.b64decode(msg["msgValue"])
                img_array = np.frombuffer(raw, dtype=np.uint8)
                return cv2.imdecode(img_array, cv2.IMREAD_COLOR)

            if isinstance(msg, np.ndarray):
                return msg

            self.logging.warning(f"[LaneAssist] Unknown mainCamera msg type: {type(msg)}")
            return None

        except Exception as e:
            self.logging.warning(f"[LaneAssist] Frame decode failed: {e}")
            return None

    # ------------------ Helper: image processing ------------------ #
    def _region_of_interest(self, image):
        """
        We select only a trapezoidal area in the bottom part of the image
        where we expect the lane to be.
        We use coordinates RELATIVE to the image dimensions.
        """
        height, width = image.shape[:2]

        # points in (x, y)
        bottom_left = (int(0.05 * width), height)
        top_left = (int(0.35 * width), int(0.6 * height))
        top_right = (int(0.65 * width), int(0.6 * height))
        bottom_right = (int(0.95 * width), height)

        polygons = np.array([[bottom_left, top_left, top_right, bottom_right]])
        mask = np.zeros_like(image)
        cv2.fillPoly(mask, polygons, (255, 255, 255))
        masked_image = cv2.bitwise_and(image, mask)
        return masked_image

    def _preprocess_for_edges(self, frame):
        """Gray -> blur -> threshold -> Canny."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        _, thresh = cv2.threshold(blur, 150, 255, cv2.THRESH_BINARY)
        edges = cv2.Canny(thresh, 50, 150)
        return edges

    def _compute_lane_error(self, frame):
        """
        Detects the lane lines and computes the lateral error.

        return:
            error_px (float): car_center_x - lane_center_x
                >0 => the car is to the right of the lane
                <0 => the car is to the left of the lane
            None if we can't find the lane
        """
        height, width = frame.shape[:2]
        car_center_x = width / 2.0

        edges = self._preprocess_for_edges(frame)
        roi = self._region_of_interest(edges)

        # Hough transform for lines
        lines = cv2.HoughLinesP(
            roi,
            rho=1,
            theta=np.pi / 180,
            threshold=50,
            minLineLength=40,
            maxLineGap=50,
        )

        if lines is None:
            # no lines found
            return None

        left_fit = []
        right_fit = []

        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)

            # filter nearly-horizontal lines (not interesting for us)
            if y2 == y1:
                continue

            params = np.polyfit((x1, x2), (y1, y2), 1)
            slope = params[0]
            intercept = params[1]

            # left lines have negative slope (in the image coordinate system)
            if slope < 0:
                left_fit.append((slope, intercept))
            else:
                right_fit.append((slope, intercept))

        if not left_fit or not right_fit:
            # we don't have both lane edges
            return None

        left_avg = np.mean(left_fit, axis=0)
        right_avg = np.mean(right_fit, axis=0)

        def make_line_coords(slope, intercept):
            y1 = height
            y2 = int(height * 0.6)
            x1 = int((y1 - intercept) / slope)
            x2 = int((y2 - intercept) / slope)
            return x1, y1, x2, y2

        x1L, y1L, x2L, y2L = make_line_coords(*left_avg)
        x1R, y1R, x2R, y2R = make_line_coords(*right_avg)

        # lane center at the bottom of the image
        lane_center_x = (x1L + x1R) / 2.0
        error_px = car_center_x - lane_center_x

        if self.debugging:
            self.logging.info(
                f"[LaneAssist] lane_center_x={lane_center_x:.1f}, car_center_x={car_center_x:.1f}, error={error_px:.1f}"
            )

        return error_px

    # ------------------ Fuzzy controller ------------------ #
    def _fuzzy_steer(self, error_px, speed_cm_s):
        """
        Simplified fuzzy controller:
        - if we have skfuzzy, we use the memberships from your example
        - if NOT, we use a simple proportional control as fallback
        """
        # convert speed from cm/s to km/h
        speed_kmh = speed_cm_s * 0.036

        # limit the error to the interval [-150, 150]
        if error_px is None:
            error_px = 0.0
        error_clipped = max(-150.0, min(150.0, float(error_px)))

        # try to use skfuzzy
        try:
            import skfuzzy as fuzz
            from skfuzzy import control as ctrl
        except ImportError:
            # simple fallback: proportional control
            k_p = 0.004  # 0.4% of the error in normalized degrees
            correction = -k_p * error_clipped  # minus: if the lane is on the left, steer left
            return max(-0.2, min(0.2, correction))

        # universes
        error = ctrl.Antecedent(np.arange(-150, 151, 1), "error")
        speed = ctrl.Antecedent(np.arange(0, 61, 1), "speed")
        correction = ctrl.Consequent(np.arange(-0.2, 0.21, 0.001), "correction")

        # membership for error
        error["very_Low_L"] = fuzz.trimf(error.universe, [-150, -150, -64.36])
        error["Left"] = fuzz.trimf(error.universe, [-99.5, -67.61, 0])
        error["Center"] = fuzz.trimf(error.universe, [-49.1, 0.325, 49.73])
        error["right"] = fuzz.trimf(error.universe, [0.325, 57.2, 100])
        error["very_high_R"] = fuzz.trimf(error.universe, [64.03, 150, 150])

        # membership for speed
        speed["slow"] = fuzz.trimf(speed.universe, [0, 0, 12.59])
        speed["medium"] = fuzz.trimf(speed.universe, [0.077, 13.09, 26.59])
        speed["high"] = fuzz.trimf(speed.universe, [17, 24.7, 35.4])
        speed["very_high"] = fuzz.trapmf(speed.universe, [26.8, 39.9, 49.6, 50])

        # membership for correction
        correction["left"] = fuzz.trapmf(correction.universe, [-0.2, -0.17, -0.1, -0.028])
        correction["slightly_left"] = fuzz.trimf(correction.universe, [-0.05, -0.035, -0.011])
        correction["no_steering"] = fuzz.trimf(correction.universe, [-0.02, 0, 0.02])
        correction["slightly_right"] = fuzz.trimf(correction.universe, [0.011, 0.035, 0.05])
        correction["right"] = fuzz.trapmf(correction.universe, [0.028, 0.1, 0.17, 0.2])

        # rules (simplified, like in the example)
        rules = [
            ctrl.Rule(error["Left"] & speed["slow"], correction["slightly_right"]),
            ctrl.Rule(error["Left"] & speed["medium"], correction["slightly_right"]),
            ctrl.Rule(error["Left"] & speed["high"], correction["slightly_right"]),
            ctrl.Rule(error["Left"] & speed["very_high"], correction["slightly_right"]),
            ctrl.Rule(error["right"] & speed["slow"], correction["slightly_left"]),
            ctrl.Rule(error["right"] & speed["medium"], correction["slightly_left"]),
            ctrl.Rule(error["right"] & speed["high"], correction["slightly_left"]),
            ctrl.Rule(error["right"] & speed["very_high"], correction["slightly_left"]),
            ctrl.Rule(error["Center"] & speed["slow"], correction["no_steering"]),
            ctrl.Rule(error["Center"] & speed["medium"], correction["no_steering"]),
            ctrl.Rule(error["Center"] & speed["high"], correction["no_steering"]),
            ctrl.Rule(error["Center"] & speed["very_high"], correction["no_steering"]),
        ]

        controller = ctrl.ControlSystem(rules)
        sim = ctrl.ControlSystemSimulation(controller)

        sim.input["error"] = error_clipped
        sim.input["speed"] = max(0.0, min(60.0, speed_kmh))

        try:
            sim.compute()
            corr = float(sim.output["correction"])
        except Exception as e:
            self.logging.warning(f"[LaneAssist] Fuzzy compute error: {e}")
            corr = 0.0

        # make sure we're in [-0.2, 0.2]
        corr = max(-0.2, min(0.2, corr))

        if self.debugging:
            self.logging.info(f"[LaneAssist] Fuzzy: error={error_clipped:.1f}, speed={speed_kmh:.1f} km/h, corr={corr:.3f}")

        return corr

    def _draw_lane_debug(self, frame):
        edges = self._preprocess_for_edges(frame)
        roi = self._region_of_interest(edges)
        lines = cv2.HoughLinesP(
            roi, 1, np.pi / 180, threshold=50, minLineLength=40, maxLineGap=50
        )
        out = frame.copy()
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line.reshape(4)
                cv2.line(out, (x1, y1), (x2, y2), (0, 255, 0), 2)
        # cv2.imshow("frame_test", out)
        return out

    def _compute_lane_offset(self, frame):
        """
        Returns an offset [-100, 100] where:
        - 0 = centered in the lane
        - >0 = the car is to the right of the center
        - <0 = the car is to the left of the center
        """
        h, w, _ = frame.shape
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)

        mask = np.zeros_like(edges)
        roi = np.array(
            [[(0, h), (0, int(h * 0.6)), (w, int(h * 0.6)), (w, h)]], np.int32
        )
        cv2.fillPoly(mask, roi, 255)
        cropped = cv2.bitwise_and(edges, mask)

        lines = cv2.HoughLinesP(
            cropped, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=50
        )

        if lines is None:
            self.logging.info("[LaneAssist] Lane not detected")
            return 0

        left_x_sum, left_cnt = 0, 0
        right_x_sum, right_cnt = 0, 0

        for line in lines:
            x1, y1, x2, y2 = line[0]
            if x2 == x1:
                continue
            slope = (y2 - y1) / (x2 - x1)
            if abs(slope) < 0.3:
                continue
            if slope < 0:
                left_x_sum += x1 + x2
                left_cnt += 2
            else:
                right_x_sum += x1 + x2
                right_cnt += 2

        if left_cnt == 0 or right_cnt == 0:
            self.logging.info("[LaneAssist] Lane not detected (one side missing)")
            return 0

        left_avg_x = left_x_sum / left_cnt
        right_avg_x = right_x_sum / right_cnt

        lane_center = (left_avg_x + right_avg_x) / 2.0
        car_center = w / 2.0

        offset_px = car_center - lane_center
        normalized = (offset_px / (w / 2.0)) * 100.0

        return int(max(-100, min(100, normalized)))

    # ------------------ MAIN LOOP ------------------ #
    def thread_work(self):
        # read state change (non-blocking)
        self.state_change_handler()

        # STEP 1: only in MANUAL (so the test is clear)
        if self.current_mode != "MANUAL":
            time.sleep(0.02)
            return

        # get the frame from serialCamera (low-res, good for debug)
        msg = self.serialCameraSubscriber.receive()
        if msg is None:
            # don't spam the log forever
            now = time.time()
            if now - self._last_no_cam_log > 1.0:
                self.logging.warning("[LaneAssist] I am not receiving serialCamera from the queue.")
                self._last_no_cam_log = now
            time.sleep(0.01)
            return

        frame = self._decode_frame(msg)
        if frame is None:
            self.logging.warning("[LaneAssist] Frame decode failed")
            return

        # draw green lines on the frame
        debug_frame = self._draw_lane_debug(frame)

        # encode the debug frame and send it to the dashboard (we use serialCamera as a "display channel")
        ok, enc = cv2.imencode(".jpg", debug_frame)
        if not ok:
            return
        b64 = base64.b64encode(enc).decode("utf-8")

        # send the overlay image to the dashboard
        # self.debug_cam_sender.send(b64)

        # (optional) also send numeric offset (so you also have the value in the report)
        offset = self._compute_lane_offset(frame)
        # self.laneKeepingSender.send(int(offset))

        time.sleep(0.05)
