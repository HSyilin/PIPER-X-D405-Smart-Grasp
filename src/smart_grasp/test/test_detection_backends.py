import cv2
import numpy as np

from smart_grasp.detection_backends import HsvBackend


def test_hsv_backend_returns_all_blue_rectangles():
    image = np.zeros((300, 400, 3), dtype=np.uint8)
    cv2.rectangle(image, (30, 50), (130, 140), (255, 0, 0), -1)
    cv2.rectangle(image, (220, 120), (350, 240), (255, 0, 0), -1)
    detections = HsvBackend(min_area=500).detect(image)
    assert len(detections) == 2
    assert all(item.class_name == "blue_block" for item in detections)
    assert all(np.count_nonzero(item.binary_mask) > 500 for item in detections)


def test_hsv_backend_rejects_small_noise():
    image = np.zeros((100, 100, 3), dtype=np.uint8)
    cv2.rectangle(image, (2, 2), (10, 10), (255, 0, 0), -1)
    assert not HsvBackend(min_area=500).detect(image)
