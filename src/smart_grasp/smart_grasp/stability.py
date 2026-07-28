"""Multi-frame object stability tracking."""

from collections import deque
from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation


@dataclass
class StabilityResult:
    stable: bool
    position_span: float
    yaw_span: float
    position: np.ndarray
    yaw: float


def _axis_center_and_deltas(yaws):
    """Return a circular center/deltas for an axis with 180-degree symmetry."""
    yaws = np.asarray(yaws, dtype=float)
    doubled = 2.0 * yaws
    center = 0.5 * np.arctan2(np.mean(np.sin(doubled)), np.mean(np.cos(doubled)))
    deltas = 0.5 * np.arctan2(
        np.sin(2.0 * (yaws - center)),
        np.cos(2.0 * (yaws - center)),
    )
    return float(center), deltas


class PoseStabilityWindow:
    def __init__(
        self, length=10, outlier_radius=0.020,
        yaw_outlier_radius=np.deg2rad(12.0),
    ):
        self.length = int(length)
        self.outlier_radius = float(outlier_radius)
        self.yaw_outlier_radius = float(yaw_outlier_radius)
        self.samples = deque(maxlen=self.length)

    def clear(self):
        self.samples.clear()

    def add(self, position, yaw):
        self.samples.append((np.asarray(position, dtype=float), float(yaw)))

    def result(self, max_position_span=0.015, max_yaw_span=np.deg2rad(5.0)):
        if not self.samples:
            return StabilityResult(
                False, float("inf"), float("inf"), np.zeros(3), 0.0
            )
        positions = np.asarray([sample[0] for sample in self.samples])
        yaws = np.asarray([sample[1] for sample in self.samples])
        median = np.median(positions, axis=0)
        yaw_center, _ = _axis_center_and_deltas(yaws)
        if len(self.samples) < self.length:
            return StabilityResult(
                False, float("inf"), float("inf"), median, yaw_center
            )

        required = max(3, int(0.8 * self.length))
        inliers = np.linalg.norm(positions - median, axis=1) <= self.outlier_radius
        if np.count_nonzero(inliers) < required:
            return StabilityResult(
                False, float("inf"), float("inf"), median, yaw_center
            )
        positions = positions[inliers]
        yaws = yaws[inliers]
        yaw_center, yaw_deltas = _axis_center_and_deltas(yaws)
        yaw_inliers = np.abs(yaw_deltas) <= self.yaw_outlier_radius
        if np.count_nonzero(yaw_inliers) < required:
            return StabilityResult(
                False, float("inf"), float("inf"),
                np.median(positions, axis=0), yaw_center,
            )

        positions = positions[yaw_inliers]
        yaws = yaws[yaw_inliers]
        yaw_center, yaw_deltas = _axis_center_and_deltas(yaws)
        position_span = float(np.max(np.ptp(positions, axis=0)))
        yaw_span = float(np.ptp(yaw_deltas))
        return StabilityResult(
            position_span <= max_position_span and yaw_span <= max_yaw_span,
            position_span,
            yaw_span,
            np.median(positions, axis=0),
            yaw_center,
        )


def sample_spans(samples):
    """Return maximum Cartesian-axis span and pairwise orientation span."""
    positions = np.asarray([sample[0] for sample in samples], dtype=float)
    position_span = float(np.max(np.ptp(positions, axis=0)))
    rotations = Rotation.from_quat([sample[1] for sample in samples])
    maximum_angle = 0.0
    for left in range(len(rotations)):
        for right in range(left + 1, len(rotations)):
            angle = (rotations[left].inv() * rotations[right]).magnitude()
            maximum_angle = max(maximum_angle, float(angle))
    return position_span, np.degrees(maximum_angle)
