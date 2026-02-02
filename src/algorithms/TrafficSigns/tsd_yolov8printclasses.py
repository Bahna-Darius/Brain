import os
from pathlib import Path

class TrafficSignDetectorYOLOv8:
    """
    Small wrapper around Ultralytics YOLOv8.
    Returns detections as a list of dicts:
      { "cls": int, "conf": float, "xyxy": (x1,y1,x2,y2) }
    """

    def __init__(self, model_path: str, conf: float = 0.4, imgsz: int = 416):
        self.conf = conf
        self.imgsz = imgsz

        try:
            from ultralytics import YOLO
        except Exception as e:
            raise RuntimeError(
                "Ultralytics is not installed. Install with: pip install ultralytics"
            ) from e

        mp = Path(model_path)
        if not mp.exists():
            raise FileNotFoundError(f"Model not found: {mp}")

        self.model = YOLO(str(mp))

        # Print model class list
        print("\n[TrafficSignDetector] Model classes:")
        for k, v in self.model.names.items():
            print(f"  {k} -> {v}")
        print()


    def detect(self, bgr_frame):
        """
        bgr_frame: numpy array (H,W,3) in BGR (OpenCV)
        """
        results = self.model(bgr_frame, imgsz=self.imgsz, conf=self.conf, verbose=False)[0]

        detections = []
        if results.boxes is None:
            return detections

        for box in results.boxes:
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            detections.append({"cls": cls, "conf": conf, "xyxy": (x1, y1, x2, y2)})

        return detections
