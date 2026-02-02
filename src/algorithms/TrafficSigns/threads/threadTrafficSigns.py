import base64
import cv2
import numpy as np
import os
from pathlib import Path

from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.allMessages import mainCamera, TrafficSignDetections

from src.algorithms.TrafficSigns.tsd_yolov8 import TrafficSignDetectorYOLOv8


class threadTrafficSigns(ThreadWithStop):
    def __init__(self, queueList, logging, debugging=False, conf=0.4, imgsz=416, every_n_frames=5):
        super(threadTrafficSigns, self).__init__()
        self.queueList = queueList
        self.logger = logging
        self.debugging = debugging

        self.every_n_frames = every_n_frames
        self.counter = 0

        # Subscribe to camera frames
        self.camSub = messageHandlerSubscriber(self.queueList, mainCamera, "lastOnly", True)

        # Publisher for detections (Dashboard will forward it)
        self.detSender = messageHandlerSender(self.queueList, TrafficSignDetections)

        # Load model
        model_path = Path(__file__).resolve().parents[1] / "models" / "tsd_best.pt"
        self.detector = TrafficSignDetectorYOLOv8(str(model_path), conf=conf, imgsz=imgsz)

    def run(self):
        while not self._event.is_set():
            msg = self.camSub.receive()
            if msg is None:
                self._event.wait(0.01)
                continue

            self.counter += 1
            if self.counter % self.every_n_frames != 0:
                continue

            try:
                # msg is base64 string of jpg
                image_data = base64.b64decode(msg)
                np_img = np.frombuffer(image_data, dtype=np.uint8)
                frame = cv2.imdecode(np_img, cv2.IMREAD_COLOR)
                if frame is None:
                    continue

                dets = self.detector.detect(frame)

                # Send only top few to reduce spam
                payload = {
                    "count": len(dets),
                    "detections": dets[:5],
                }
                self.detSender.send(payload)

                if self.debugging and len(dets) > 0:
                    self.logger.info(f"[TSD] {payload}")

            except Exception as e:
                if self.debugging:
                    self.logger.warning(f"[TSD] error: {e}")
