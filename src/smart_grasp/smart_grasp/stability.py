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


class PoseStabilityWindow:
    def __init__(self, length=10, outlier_radius=0.020):
        self.length = int(length)
        self.outlier_radius = float(outlier_radius)
        self.samples = deque(maxlen=self.length)

    def clear(self):
        self.samples.clear()

    def add(self, position, yaw):
        self.samples.append((np.asarray(position, dtype=float), float(yaw)))

    def result(self, max_position_span=0.015, max_yaw_span=np.deg2rad(5.0)):
        if len(self.samples) < self.length:
            return StabilityResult(False, float("inf"), float("inf"))
        positions = np.asarray([sample[0] for sample in self.samples])
        median = np.median(positions, axis=0)
        inliers = np.linalg.norm(positions - median, axis=1) <= self.outlier_radius
        if np.count_nonzero(inliers) < max(3, int(0.8 * self.length)):
            return StabilityResult(False, float("inf"), float("inf"))
        positions = positions[inliers]
        yaws = np.unwrap(np.asarray([sample[1] for sample in self.samples])[inliers] * 2.0) / 2.0
        position_span = float(np.max(np.ptp(positions, axis=0)))
        yaw_span = float(np.ptp(yaws))
        return StabilityResult(
            position_span <= max_position_span and yaw_span <= max_yaw_span,
            position_span,
            yaw_span,
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
