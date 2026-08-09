import numpy as np

from smart_grasp.depth_geometry import matrix_from_transform
from smart_grasp.handeye_tf_node import camera_mount_matrix, piper_x_base_to_tcp_matrix


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


def test_piper_x_fk_matches_recorded_observation_pose():
    joints = [
        0.0,
        0.195633956,
        -0.481920313,
        0.945113243,
        -0.117355939,
        0.0,
    ]
    base_to_tcp = piper_x_base_to_tcp_matrix(joints)

    recorded_position = np.array([0.068635, 0.004128, 0.276414])
    # The recorded quaternion may have either sign; compare as a rotation matrix.
    recorded_tcp = matrix_from_transform(
        recorded_position,
        [-0.639503, 0.599765, -0.301731, 0.374533],
    )

    assert np.allclose(base_to_tcp[:3, 3], recorded_tcp[:3, 3], atol=3e-4)
    assert np.allclose(base_to_tcp[:3, :3], recorded_tcp[:3, :3], atol=5e-5)
