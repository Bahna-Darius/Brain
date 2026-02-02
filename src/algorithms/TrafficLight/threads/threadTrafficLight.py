# Copyright (c) 2026, NovaVision
# All rights reserved.

import base64
import time
import cv2
import numpy as np

from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import serialCamera, TrafficLight
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender

from src.algorithms.TrafficLight.simple_detector import SimpleTLDetector


class threadTrafficLight(ThreadWithStop):
    """Runs traffic light detection on camera frames.

    It subscribes to `mainCamera` (base64 JPEG) and publishes `TrafficLight` dict.
    """

    def __init__(
        self,
        queuesList,
        logger,
        debugging: bool,
        inference_hz: float = 8.0,
        skip_frames: int = 3,
    ):
        super(threadTrafficLight, self).__init__(pause=0.001)
        self.queuesList = queuesList
        self.logger = logger
        self.debugging = debugging

        self.inference_period = 1.0 / max(1.0, float(inference_hz))
        self.skip_frames = int(max(0, skip_frames))
        self._frame_counter = 0
        self._last_infer_ts = 0.0
        self._last_print = 0.0

        self.frameSubscriber = messageHandlerSubscriber(self.queuesList, serialCamera, "lastOnly", True)
        self.sender = messageHandlerSender(self.queuesList, TrafficLight)

        self.detector = SimpleTLDetector()
        if self.debugging:
            print(f"[TrafficLight] backend={self.detector.backend_name} model={self.detector.model_path}")

    def _decode_frame(self, b64_jpg: str):
        try:
            jpg_bytes = base64.b64decode(b64_jpg)
            arr = np.frombuffer(jpg_bytes, dtype=np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            return frame
        except Exception:
            return None

    def thread_work(self):
        # rate-limit inference
        now = time.time()
        if (now - self._last_infer_ts) < self.inference_period:
            return

        if not self.frameSubscriber.is_data_in_pipe():
            return

        b64 = self.frameSubscriber.receive()
        if not b64:
            return

        self._frame_counter += 1
        if self.skip_frames and (self._frame_counter % (self.skip_frames + 1)) != 1:
            return

        frame = self._decode_frame(b64)
        if frame is None:
            return


        ### DEBUG: ####
        result = self.detector.detect(frame)

        if result is None:
            return

        state, conf, bbox = result

        nowp = time.time()
        if nowp - self._last_print > 1.0:
            print(f"[TL] {state} {conf:.2f}")
            self._last_print = nowp


        payload = {
            "state": state,
            "confidence": float(conf),
            "bbox": list(bbox),
            "source": "simple",
            "timestamp": time.time()
        }

        self.sender.send(payload)
