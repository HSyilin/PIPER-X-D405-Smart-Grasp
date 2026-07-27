import numpy as np
from scipy.spatial.transform import Rotation as R

from smart_grasp.grasp_geometry import tcp_position_for_grasp_center


def test_tcp_position_for_identity_tool_offset():
    tcp = tcp_position_for_grasp_center(
        [0.40, 0.10, 0.20],
        [0.0, 0.0, 0.0, 1.0],
        [0.0, 0.0, 0.1425],
    )
    np.testing.assert_allclose(tcp, [0.40, 0.10, 0.0575], atol=1e-9)


def test_tcp_position_respects_tcp_orientation():
    quaternion = R.from_euler("y", np.pi).as_quat()
    tcp = tcp_position_for_grasp_center(
        [0.40, 0.10, 0.20],
        quaternion,
        [0.0, 0.0, 0.1425],
    )
    np.testing.assert_allclose(tcp, [0.40, 0.10, 0.3425], atol=1e-9)
