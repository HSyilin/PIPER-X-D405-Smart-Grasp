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


@dataclass
class SizeStabilityResult:
    stable: bool
    median_size: np.ndarray
    size_span: float


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


class SizeStabilityWindow:
    def __init__(self, length=10, outlier_tolerance=0.012):
        self.length = int(length)
        self.outlier_tolerance = float(outlier_tolerance)
        self.samples = deque(maxlen=self.length)

    def clear(self):
        self.samples.clear()

    def add(self, size):
        self.samples.append(np.asarray(size, dtype=float))

    def result(self, max_size_span=0.015):
        if not self.samples:
            return SizeStabilityResult(False, np.zeros(3), float("inf"))

        sizes = np.asarray(self.samples)
        median = np.median(sizes, axis=0)
        inliers = np.max(np.abs(sizes - median), axis=1) <= self.outlier_tolerance
        required = max(3, int(0.8 * self.length))
        if len(self.samples) < self.length or np.count_nonzero(inliers) < required:
            return SizeStabilityResult(False, median, float("inf"))

        sizes = sizes[inliers]
        median = np.median(sizes, axis=0)
        size_span = float(np.max(np.ptp(sizes, axis=0)))
        return SizeStabilityResult(
            size_span <= float(max_size_span), median, size_span
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
