"""Interchangeable 2-D instance-mask detection backends."""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple

import cv2
import numpy as np


@dataclass
class InstanceMask:
    class_name: str
    confidence: float
    binary_mask: np.ndarray
    bounding_rect: Tuple[int, int, int, int]
    contour: np.ndarray
    angle_deg: float
    rejection_reason: str = ""


class DetectionBackend(ABC):
    @abstractmethod
    def detect(self, bgr_image: np.ndarray) -> List[InstanceMask]:
        """Return every instance accepted by the 2-D detector."""


class HsvBackend(DetectionBackend):
    def __init__(
        self,
        lower=(90, 80, 50),
        upper=(135, 255, 255),
        min_area=500.0,
        min_solidity=0.85,
        min_rectangularity=0.65,
    ):
        self.lower = np.asarray(lower, dtype=np.uint8)
        self.upper = np.asarray(upper, dtype=np.uint8)
        self.min_area = float(min_area)
        self.min_solidity = float(min_solidity)
        self.min_rectangularity = float(min_rectangularity)

    def detect(self, bgr_image: np.ndarray) -> List[InstanceMask]:
        hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower, self.upper)
        mask = cv2.medianBlur(mask, 5)
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_OPEN, np.ones((3, 3), dtype=np.uint8)
        )
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_CLOSE, np.ones((7, 7), dtype=np.uint8)
        )
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        detections = []
        image_area = float(bgr_image.shape[0] * bgr_image.shape[1])
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < self.min_area:
                continue
            hull_area = float(cv2.contourArea(cv2.convexHull(contour)))
            rect = cv2.minAreaRect(contour)
            rect_area = float(rect[1][0] * rect[1][1])
            if hull_area <= 0.0 or rect_area <= 0.0:
                continue
            solidity = area / hull_area
            rectangularity = area / rect_area
            if solidity < self.min_solidity or rectangularity < self.min_rectangularity:
                continue
            instance_mask = np.zeros(mask.shape, dtype=np.uint8)
            cv2.drawContours(instance_mask, [contour], -1, 255, thickness=cv2.FILLED)
            confidence = min(
                0.99,
                0.45 * solidity
                + 0.45 * rectangularity
                + 0.10 * min(1.0, area / max(self.min_area, image_area * 0.02)),
            )
            detections.append(
                InstanceMask(
                    class_name="blue_block",
                    confidence=float(confidence),
                    binary_mask=instance_mask,
                    bounding_rect=cv2.boundingRect(contour),
                    contour=contour,
                    angle_deg=float(rect[2]),
                )
            )
        return detections


class YoloSegBackend(DetectionBackend):
    def __init__(self, model_path, target_class="blue_block", confidence=0.70):
        path = Path(model_path).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f"YOLO-Seg model does not exist: {path}")
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError("ultralytics is required for detector_backend=yolo_seg") from exc
        self.model = YOLO(str(path))
        self.target_class = str(target_class)
        self.confidence = float(confidence)

    def detect(self, bgr_image: np.ndarray) -> List[InstanceMask]:
        results = self.model.predict(
            source=bgr_image, conf=self.confidence, verbose=False, retina_masks=True
        )
        detections = []
        height, width = bgr_image.shape[:2]
        for result in results:
            if result.boxes is None or result.masks is None:
                continue
            masks = result.masks.data.cpu().numpy()
            for index, box in enumerate(result.boxes):
                class_id = int(box.cls[0])
                class_name = str(self.model.names[class_id])
                if class_name != self.target_class:
                    continue
                raw_mask = masks[index]
                resized = cv2.resize(raw_mask, (width, height), interpolation=cv2.INTER_NEAREST)
                binary = np.where(resized > 0.5, 255, 0).astype(np.uint8)
                contours, _ = cv2.findContours(
                    binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
                )
                if not contours:
                    continue
                contour = max(contours, key=cv2.contourArea)
                detections.append(
                    InstanceMask(
                        class_name=self.target_class,
                        confidence=float(box.conf[0]),
                        binary_mask=binary,
                        bounding_rect=cv2.boundingRect(contour),
                        contour=contour,
                        angle_deg=float(cv2.minAreaRect(contour)[2]),
                    )
                )
        return detections
