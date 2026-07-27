import numpy as np

from smart_grasp.stability import PoseStabilityWindow, SizeStabilityWindow


def test_stability_rejects_until_window_full():
    window = PoseStabilityWindow(length=10)
    for index in range(9):
        window.add([0.3 + index * 0.0001, 0.0, 0.1], 0.01)
    assert not window.result().stable
    window.add([0.301, 0.0, 0.1], 0.011)
    assert window.result().stable


def test_stability_rejects_wide_yaw_span():
    window = PoseStabilityWindow(length=10)
    for yaw in np.linspace(0.0, np.deg2rad(10.0), 10):
        window.add([0.3, 0.0, 0.1], yaw)
    assert not window.result().stable


def test_size_stability_uses_median_and_rejects_outlier():
    window = SizeStabilityWindow(length=10, outlier_tolerance=0.015)
    samples = [
        [0.0950, 0.0680, 0.0310],
        [0.0960, 0.0675, 0.0320],
        [0.0940, 0.0690, 0.0315],
        [0.0970, 0.0685, 0.0305],
        [0.0955, 0.0670, 0.0310],
        [0.0965, 0.0700, 0.0325],
        [0.0935, 0.0680, 0.0300],
        [0.0950, 0.0695, 0.0315],
        [0.0960, 0.0685, 0.0310],
        [0.1200, 0.0900, 0.0500],
    ]
    for size in samples:
        window.add(size)

    result = window.result(max_size_span=0.015)
    assert result.stable
    assert np.allclose(result.median_size, [0.09525, 0.0685, 0.0310], atol=0.001)
    assert result.size_span < 0.015


def test_size_stability_waits_for_full_window():
    window = SizeStabilityWindow(length=5)
    for _ in range(4):
        window.add([0.095, 0.068, 0.031])
    assert not window.result().stable
