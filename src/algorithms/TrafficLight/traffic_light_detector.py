# Copyright (c) 2026, NovaVision
# All rights reserved.

import cv2
import numpy as np


class SimpleTLDetector:

    def __init__(self):

        # ROI settings (adjustable)
        self.roi_y = 0.05
        self.roi_h = 0.35

        self.roi_x = 0.25
        self.roi_w = 0.5


    def detect(self, frame):

        h, w = frame.shape[:2]

        y1 = int(self.roi_y * h)
        y2 = int((self.roi_y + self.roi_h) * h)

        x1 = int(self.roi_x * w)
        x2 = int((self.roi_x + self.roi_w) * w)

        roi = frame[y1:y2, x1:x2]

        if roi.size == 0:
            return "unknown", 0.0, (0,0,0,0)

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # RED
        r1 = cv2.inRange(hsv,(0,80,80),(10,255,255))
        r2 = cv2.inRange(hsv,(160,80,80),(180,255,255))
        red = r1 | r2

        # GREEN
        green = cv2.inRange(hsv,(40,70,70),(90,255,255))

        # YELLOW
        yellow = cv2.inRange(hsv,(15,70,70),(35,255,255))

        total = roi.shape[0] * roi.shape[1]

        r = cv2.countNonZero(red) / total
        g = cv2.countNonZero(green) / total
        y = cv2.countNonZero(yellow) / total

        scores = {
            "red": r,
            "green": g,
            "yellow": y
        }

        state = max(scores, key=scores.get)
        conf = scores[state]

        if conf < 0.01:
            state = "unknown"

        bbox = (x1, y1, x2, y2)

        return state, conf, bbox
