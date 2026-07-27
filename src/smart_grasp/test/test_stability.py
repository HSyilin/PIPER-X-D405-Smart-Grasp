import numpy as np

from smart_grasp.stability import PoseStabilityWindow


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
