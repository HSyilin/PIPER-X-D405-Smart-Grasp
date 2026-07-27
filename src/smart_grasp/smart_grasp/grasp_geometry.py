"""Geometry helpers for the minimal top-down grasp pipeline."""

import numpy as np
from scipy.spatial.transform import Rotation as R


def pose_to_matrix(position, quaternion):
    """Return T_parent_child from xyz and xyzw quaternion."""
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = R.from_quat(quaternion).as_matrix()
    transform[:3, 3] = np.asarray(position, dtype=float)
    return transform


def tcp_position_for_grasp_center(
    grasp_center_position,
    tcp_quaternion,
    tcp_to_grasp_translation,
):
    """Convert a desired grasp-center position into the TCP/flange position.

    ``tcp_to_grasp_translation`` is expressed in the TCP frame.  Piper-X's
    current URDF places the nominal finger grasp center at approximately
    [0, 0, 0.1425] m from the flange/TCP.
    """
    grasp_center = np.asarray(grasp_center_position, dtype=float)
    tool_offset = np.asarray(tcp_to_grasp_translation, dtype=float)
    rotation = R.from_quat(tcp_quaternion).as_matrix()
    return grasp_center - rotation @ tool_offset
