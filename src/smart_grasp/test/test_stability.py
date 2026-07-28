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


def test_stability_rejects_single_yaw_outlier_but_publishes_filtered_yaw():
    window = PoseStabilityWindow(length=10, yaw_outlier_radius=np.deg2rad(12.0))
    for yaw in np.deg2rad([-1.0, 0.0, 0.5, -0.5, 1.0, 0.2, -0.2, 0.7, -0.7, 35.0]):
        window.add([0.3, 0.0, 0.1], yaw)

    result = window.result()
    assert result.stable
    assert abs(np.rad2deg(result.yaw)) < 1.0
    assert np.rad2deg(result.yaw_span) < 2.1


def test_stability_treats_opposite_axis_directions_as_equivalent():
    window = PoseStabilityWindow(length=10)
    for yaw in np.deg2rad([89.5, -90.5, 90.0, -90.0, 89.8] * 2):
        window.add([0.3, 0.0, 0.1], yaw)

    assert window.result().stable
