import numpy as np

from smart_grasp.depth_geometry import matrix_from_transform
from smart_grasp.handeye_tf_node import camera_mount_matrix


def test_camera_mount_composition_recovers_tcp_to_optical():
    calibration = {
        "tcp_to_color_optical": {
            "translation": [-0.018, -0.077, 0.053],
            "quaternion_xyzw": [-0.172, -0.005, 0.037, 0.984],
        },
        "camera_link_to_color_optical": {
            "translation": [-0.00001, 0.00001, 0.00001],
            "quaternion_xyzw": [-0.503, 0.497, -0.498, 0.501],
        },
    }
    tcp_to_link = camera_mount_matrix(calibration)
    link_entry = calibration["camera_link_to_color_optical"]
    link_to_optical = matrix_from_transform(
        link_entry["translation"], link_entry["quaternion_xyzw"]
    )
    tcp_entry = calibration["tcp_to_color_optical"]
    expected = matrix_from_transform(
        tcp_entry["translation"], tcp_entry["quaternion_xyzw"]
    )

    assert np.allclose(tcp_to_link @ link_to_optical, expected)
