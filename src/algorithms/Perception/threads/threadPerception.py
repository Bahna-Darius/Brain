# Copyright (c) 2026, NovaVision
# All rights reserved.

################ NovaVision 26.01.2026 ###############
# Explaining context, what have been done:
# - Implemented Hybrid Perception module:
#     * Camera publishes only frames (serialCamera).
#     * Perception subscribes to serialCamera, decodes once, runs ALL vision modules inside.
# - Outputs a single fused dict message: PerceptionContext.
# - No actuator commands are sent from Perception (no SpeedMotor/SteerMotor).
# - Fixed BFMC naming consistency: debugger flag + logging calls.
#####################################################

import base64
import os
import time
import traceback

import cv2
import numpy as np

from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.algorithms.TrafficLight.simple_detector import SimpleTLDetector
from src.utils.messages.allMessages import serialCameraRaw, serialCamera, PerceptionContext, Record

class threadPerception(ThreadWithStop):
    """
    Hybrid Perception thread:
      - Input: serialCamera (base64 JPEG string)
      - Processing: LaneAssist + TrafficSign YOLO
      - Output: PerceptionContext dict (fused)
    """

    def __init__(self, queueList, logger, debugger=False):
        super(threadPerception, self).__init__(pause=0.001)
        self.queueList = queueList
        self.logger = logger
        self.debugger = bool(debugger)

        # ---- Subscribers / Senders ----
        self.serialFrameSub = messageHandlerSubscriber(self.queueList, serialCameraRaw, "lastOnly", True)
        self.ctxSender = messageHandlerSender(self.queueList, PerceptionContext)
        self.serialVizSender = messageHandlerSender(self.queueList, serialCamera)

        # ---- Traffic light state ----
        self.last_tl = None
        self.last_tl_ts = 0.0
        self.tl_detector = SimpleTLDetector()

        # ---- Rates ----
        self.PERCEPTION_HZ = 10.0
        self.YOLO_EVERY_N = 5  # run YOLO at PERCEPTION_HZ / N
        self._tick = 0
        self._last_log_ts = 0.0
        self.TL_EVERY_N = 1  # run every perception tick (cheap)

        # ---- Module state ----
        self.last_lane = None
        self.last_lane_ts = 0.0

        self.last_signs = None
        self.last_signs_ts = 0.0

        # ---- Prepare algorithms ----
        self.detector = None
        self.lane_keeper = None
        self.sign_detector = None

        self._init_lane_algorithms()
        self._init_traffic_sign_detector()

        # ---- Frame cache ----
        self._last_frame = None
        self._last_frame_ts = 0.0

        # recording helper
        self.recordSub = messageHandlerSubscriber(self.queueList, Record, "lastOnly", True)
        self.recording_overlay = False
        self.overlay_writer = None
        self.overlay_path = None

    # =====================================================================================
    # Recording overlay frame
    # =====================================================================================    

    def _parse_bool(self, x):
        if x is None:
            return None
        if isinstance(x, bool):
            return x
        s = str(x).strip().lower()
        if s in ("true", "1", "yes", "on"):
            return True
        if s in ("false", "0", "no", "off"):
            return False
        return bool(x)

    # =====================================================================================
    # Init Lane Algorithms
    # =====================================================================================
    def _init_lane_algorithms(self):
        try:
            from src.algorithms.LaneAssist.detect import LaneDetection
            from src.algorithms.LaneAssist.lanekeeping import LaneKeeping as LKAlgo

            self.lane_keeper = LKAlgo()
            self.detector = LaneDetection(lk_object=self.lane_keeper)

            if self.debugger:
                self.logger.info("[Perception] LaneAssist initialized.")
        except Exception as e:
            self.detector = None
            self.lane_keeper = None
            self.logger.warning(f"[Perception] LaneAssist NOT initialized: {e}")
            if self.debugger:
                traceback.print_exc()

    # =====================================================================================
    # Init Traffic Sign Detector
    # =====================================================================================
    def _init_traffic_sign_detector(self):
        model_path = "src/algorithms/TrafficSigns/models/tsd_best.pt"
        try:
            from src.algorithms.TrafficSigns.tsd_yolov8 import TrafficSignDetectorYOLOv8

            if not os.path.exists(model_path):
                raise FileNotFoundError(f"YOLO model not found at {model_path}")

            self.sign_detector = TrafficSignDetectorYOLOv8(
                model_path=model_path,
                conf=0.4,
                imgsz=416
            )

            if self.debugger:
                self.logger.info("[Perception] TrafficSign YOLO initialized.")
        except Exception as e:
            self.sign_detector = None
            self.logger.warning(f"[Perception] TrafficSign YOLO NOT initialized: {e}")
            if self.debugger:
                traceback.print_exc()

    # =====================================================================================
    # Decode serialCamera base64 JPEG -> BGR frame
    # =====================================================================================
    def _decode_frame(self, b64_jpg: str):
        t0 = time.time()
        try:
            jpg_bytes = base64.b64decode(b64_jpg)
            arr = np.frombuffer(jpg_bytes, dtype=np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            return frame, (time.time() - t0) * 1000.0
        except Exception:
            return None, (time.time() - t0) * 1000.0

    # =====================================================================================
    # Lane inference
    # =====================================================================================
    def _run_lane(self, frame_bgr):
        t0 = time.time()

        # Always return: (lane_dict_or_None, debug_frame_or_None, lane_ms)
        if self.detector is None or self.lane_keeper is None:
            return None, None, (time.time() - t0) * 1000.0

        debug_frame = None

        try:
            lane_results = self.detector.lanes_detection(frame_bgr.copy())

            # If something goes wrong and detector returns None
            if lane_results is None:
                lane = {
                    "steer": 0,
                    "confidence": 0.0,
                    "trust_left": False,
                    "trust_right": False,
                    "status": "NO_DETECTION",
                    "timestamp": time.time(),
                }
                return lane, frame_bgr, (time.time() - t0) * 1000.0

            steer_angle, debug_frame = self.lane_keeper.lane_keeping(lane_results)

            trust_left = bool(lane_results.get("trust_left", False))
            trust_right = bool(lane_results.get("trust_right", False))

            if trust_left and trust_right:
                conf = 1.0
            elif trust_left or trust_right:
                conf = 0.6
            else:
                conf = 0.2

            lane = {
                "steer": int(steer_angle),
                "confidence": float(conf),
                "trust_left": trust_left,
                "trust_right": trust_right,
                "status": "ACTIVE",
                "timestamp": time.time(),
            }

            # debug_frame can be None if lane_keeping returns None
            if debug_frame is None:
                debug_frame = lane_results.get("frame", frame_bgr)

            return lane, debug_frame, (time.time() - t0) * 1000.0

        except Exception:
            if self.debugger:
                traceback.print_exc()

            lane = {
                "steer": 0,
                "confidence": 0.0,
                "trust_left": False,
                "trust_right": False,
                "status": "CRASH",
                "timestamp": time.time(),
            }
            return lane, frame_bgr, (time.time() - t0) * 1000.0


    # =====================================================================================
    # Traffic sign inference (throttled)
    # =====================================================================================
    def _run_signs(self, frame_bgr):
        t0 = time.time()

        if self.sign_detector is None:
            return None, (time.time() - t0) * 1000.0

        try:
            dets = self.sign_detector.detect(frame_bgr) or []
            dets = dets[:5]
            # if self.debugger and dets:
            #     d0 = dets[0]
            #     self.logger.info(f"[TSD DEBUG] sample det keys={list(d0.keys())} det0={d0}")


            # --- stop sign check: supports ---
            STOP_ID = 8                 

            stop_seen = False
            for d in dets:
                cls = d.get("cls", None)
                name = str(d.get("name", d.get("class", ""))).lower()

                if (cls is not None and int(cls) == STOP_ID):
                    stop_seen = True
                    break

            signs = {
                "timestamp": time.time(),
                "count": int(len(dets)),
                "detections": dets,
                "stop_sign": bool(stop_seen),
                "status": "ACTIVE",
            }
            return signs, (time.time() - t0) * 1000.0

        except Exception:
            if self.debugger:
                traceback.print_exc()
            signs = {
                "timestamp": time.time(),
                "count": 0,
                "detections": [],
                "stop_sign": False,
                "status": "CRASH",
            }
            return signs, (time.time() - t0) * 1000.0


    # =====================================================================================
    # Main loop
    # =====================================================================================
    def thread_work(self):
        period = 1.0 / self.PERCEPTION_HZ
        start = time.time()

        # ---- Grab latest frame (lastOnly) ----
        b64_frame = self.serialFrameSub.receive()
        decode_ms = 0.0
        if b64_frame is not None:
            frame, decode_ms = self._decode_frame(b64_frame)
            if frame is not None:
                self._last_frame = frame
                self._last_frame_ts = time.time()

        if self._last_frame is None:
            time.sleep(0.05)
            return
        
        # ---- Handle overlay recording ----
        rec = self.recordSub.receive()
        rec_bool = self._parse_bool(rec)
        if rec_bool is not None and rec_bool != self.recording_overlay:
            self.recording_overlay = rec_bool

            if not self.recording_overlay:
                if self.overlay_writer is not None:
                    self.overlay_writer.release()
                    self.overlay_writer = None
                if self.debugger:
                    self.logger.info("[Perception] Overlay recording STOPPED")
            else:
                self.overlay_path = f"lane_overlay_{int(time.time())}.avi"
                if self.debugger:
                    self.logger.info(f"[Perception] Overlay recording STARTED -> {self.overlay_path}")

        self._tick += 1
        # ---- Run lane every cycle ----
        lane, lane_debug, lane_ms = self._run_lane(self._last_frame)

        # ---- Save overlay frame if recording ----
        if self.recording_overlay and lane_debug is not None:
            if self.overlay_writer is None:
                h, w = lane_debug.shape[:2]
                fourcc = cv2.VideoWriter_fourcc(*"XVID")
                self.overlay_writer = cv2.VideoWriter(self.overlay_path, fourcc, self.PERCEPTION_HZ, (w, h))

            self.overlay_writer.write(lane_debug)


        if lane is not None:
            self.last_lane = lane
            self.last_lane_ts = float(lane.get("timestamp", time.time()))

        # ---- Send overlay frame to dashboard ----
        frame_to_send = lane_debug if lane_debug is not None else self._last_frame

        ok, enc = cv2.imencode(".jpg", frame_to_send)
        if ok:
            b64 = base64.b64encode(enc).decode("utf-8")
            self.serialVizSender.send(b64)


        # ---- Run YOLO every N cycles ----
        signs_ms = 0.0
        if (self._tick % self.YOLO_EVERY_N) == 0:
            signs, signs_ms = self._run_signs(self._last_frame)
            if signs is not None:
                self.last_signs = signs
                self.last_signs_ts = float(signs.get("timestamp", time.time()))

        tl_ms = 0.0
        if (self._tick % self.TL_EVERY_N) == 0:
            t0 = time.time()
            try:
                state, conf, bbox = self.tl_detector.detect(self._last_frame)
                self.last_tl = {
                    "timestamp": time.time(),
                    "state": state,
                    "confidence": float(conf),
                    "bbox": list(bbox),
                    "source": "simple",
                    "status": "ACTIVE",
                }
                self.last_tl_ts = float(self.last_tl["timestamp"])
            except Exception:
                if self.debugger:
                    traceback.print_exc()
                self.last_tl = {
                    "timestamp": time.time(),
                    "state": "unknown",
                    "confidence": 0.0,
                    "bbox": [0, 0, 0, 0],
                    "source": "simple",
                    "status": "CRASH",
                }
                self.last_tl_ts = float(self.last_tl["timestamp"])
            tl_ms = (time.time() - t0) * 1000.0

        # ---- Build fused context ----
        now = time.time()
        ctx = {
            "timestamp": now,
            "frame_ts": float(self._last_frame_ts),
            "lane": self.last_lane,
            "traffic_signs": self.last_signs,
            "traffic_light": self.last_tl,
            "age": {
                "lane_s": None if self.last_lane_ts == 0 else float(now - self.last_lane_ts),
                "signs_s": None if self.last_signs_ts == 0 else float(now - self.last_signs_ts),
                "tl_s": None if self.last_tl_ts == 0 else float(now - self.last_tl_ts),
            },
            "debug": {
                "frame_decode_ms": float(decode_ms),
                "lane_ms": float(lane_ms),
                "tsd_ms": float(signs_ms),
                "tick": int(self._tick),
                "tl_ms": float(tl_ms),
            }
        }

        # ---- Publish fused context ----
        self.ctxSender.send(ctx)
        # if self.debugger and self.last_signs and self.last_signs.get("stop_sign", False):
        #     self.logger.warning("[Perception] ✅ STOP SIGN DETECTED!")
        #     # optional: show what it thinks it saw
        #     self.logger.warning(f"[Perception] stop dets: {self.last_signs.get('detections', [])}")


        # ---- Print something periodically so you SEE it's alive ----
        if self.debugger and (now - self._last_log_ts) > 1.0:
            self._last_log_ts = now
            tl_state = ctx["traffic_light"]["state"] if ctx.get("traffic_light") else None
            tl_conf  = ctx["traffic_light"]["confidence"] if ctx.get("traffic_light") else None
            lane_dbg = ctx["lane"]["steer"] if ctx["lane"] else None
            conf_dbg = ctx["lane"]["confidence"] if ctx["lane"] else None
            self.logger.info(
                f"[Perception] tick={self._tick} lane_steer={lane_dbg} lane_conf={conf_dbg} "
                f"tl={tl_state} tl_conf={tl_conf}"
            )

        # ---- Rate limit ----
        elapsed = time.time() - start
        time.sleep(max(0.0, period - elapsed))
