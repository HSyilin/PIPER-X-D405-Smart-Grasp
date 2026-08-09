"""Depth conversion, robust 3-D geometry, and grasp-pose helpers."""

from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np
from scipy.spatial.transform import Rotation


@dataclass
class OrientedBox:
    center: np.ndarray
    size: np.ndarray
    short_axis: np.ndarray
    long_axis: np.ndarray
    yaw: float
    top_z: float
    table_z: Optional[float]


def depth_to_meters(depth: np.ndarray, encoding: str, depth_scale=0.001):
    if encoding == "16UC1":
        return depth.astype(np.float32) * float(depth_scale)
    if encoding == "32FC1":
        return depth.astype(np.float32)
    raise ValueError(f"unsupported aligned depth encoding: {encoding}")


def masked_points(depth_m, mask, camera_matrix, minimum, maximum):
    valid_depth = np.isfinite(depth_m) & (depth_m >= minimum) & (depth_m <= maximum)
    requested = mask > 0
    valid = requested & valid_depth
    denominator = int(np.count_nonzero(requested))
    ratio = float(np.count_nonzero(valid)) / denominator if denominator else 0.0
    rows, cols = np.nonzero(valid)
    z = depth_m[rows, cols].astype(np.float64)
    fx, fy = float(camera_matrix[0, 0]), float(camera_matrix[1, 1])
    cx, cy = float(camera_matrix[0, 2]), float(camera_matrix[1, 2])
    x = (cols.astype(np.float64) - cx) * z / fx
    y = (rows.astype(np.float64) - cy) * z / fy
    return np.column_stack((x, y, z)), ratio


def transform_points(points: np.ndarray, transform: np.ndarray):
    if points.size == 0:
        return points.reshape((-1, 3))
    return points @ transform[:3, :3].T + transform[:3, 3]


def robust_filter(points: np.ndarray, radius=0.03):
    if len(points) < 5:
        return points
    median = np.median(points, axis=0)
    distance = np.linalg.norm(points - median, axis=1)
    mad = np.median(np.abs(distance - np.median(distance)))
    threshold = min(float(radius), max(0.005, np.median(distance) + 3.5 * max(mad, 1e-4)))
    return points[distance <= threshold]


def fit_horizontal_plane_z(points: np.ndarray, threshold=0.004, iterations=80):
    """Estimate a near-horizontal table plane from background points."""
    if len(points) < 50:
        return None
    rng = np.random.default_rng(7)
    sample = points
    if len(sample) > 5000:
        sample = sample[rng.choice(len(sample), 5000, replace=False)]
    best = None
    best_count = 0
    for _ in range(int(iterations)):
        tri = sample[rng.choice(len(sample), 3, replace=False)]
        normal = np.cross(tri[1] - tri[0], tri[2] - tri[0])
        norm = np.linalg.norm(normal)
        if norm < 1e-8:
            continue
        normal /= norm
        if abs(normal[2]) < 0.94:
            continue
        distance = np.abs((sample - tri[0]) @ normal)
        inlier = distance < threshold
        count = int(np.count_nonzero(inlier))
        if count > best_count:
            best_count = count
            best = sample[inlier]
    if best is None or best_count < max(30, int(0.2 * len(sample))):
        return None
    return float(np.median(best[:, 2]))


def estimate_oriented_box(
    points: np.ndarray, fixed_size, table_z: Optional[float] = None,
    orientation_surface_band=0.008,
):
    """Estimate target pose while taking all collision dimensions from configuration."""
    if len(points) < 10:
        raise ValueError("not enough points for OBB")
    size = np.asarray(fixed_size, dtype=float)
    if size.shape != (3,) or np.any(size <= 0.0):
        raise ValueError("fixed object size must contain three positive dimensions")
    top_z = float(np.percentile(points[:, 2], 95.0))
    surface_band = float(orientation_surface_band)
    if surface_band <= 0.0:
        raise ValueError("orientation surface band must be positive")
    surface_points = points[points[:, 2] >= top_z - surface_band]
    if len(surface_points) < max(10, int(0.1 * len(points))):
        surface_points = points
    xy = surface_points[:, :2].astype(np.float32)
    rectangle = cv2.minAreaRect(xy)
    center_xy = np.asarray(rectangle[0], dtype=float)
    corners = cv2.boxPoints(rectangle).astype(float)
    edges = np.roll(corners, -1, axis=0) - corners
    edge_lengths = np.linalg.norm(edges, axis=1)
    short_axis = edges[int(np.argmin(edge_lengths))]
    short_axis_length = np.linalg.norm(short_axis)
    if not np.isfinite(short_axis_length) or short_axis_length < 1e-8:
        raise ValueError("degenerate top-surface rectangle")
    short_axis /= short_axis_length
    if short_axis[0] < 0.0:
        short_axis = -short_axis
    long_axis = np.array([-short_axis[1], short_axis[0]])
    # The fixed geometry is the collision/grasp contract. Anchor its top at the
    # measured top surface so downstream reconstruction from center + size is
    # self-consistent even when the observed height differs from the profile.
    bottom_z = top_z - float(size[2])
    center = np.array([center_xy[0], center_xy[1], 0.5 * (top_z + bottom_z)])
    yaw = float(np.arctan2(short_axis[1], short_axis[0]))
    return OrientedBox(
        center=center,
        size=size.copy(),
        short_axis=np.array([short_axis[0], short_axis[1], 0.0]),
        long_axis=np.array([long_axis[0], long_axis[1], 0.0]),
        yaw=yaw,
        top_z=top_z,
        table_z=table_z,
    )


def matrix_from_transform(translation, quaternion):
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
    matrix[:3, 3] = translation
    return matrix


def quaternion_from_axes(x_axis, z_axis):
    x_axis = np.asarray(x_axis, dtype=float)
    z_axis = np.asarray(z_axis, dtype=float)
    x_axis /= np.linalg.norm(x_axis)
    z_axis /= np.linalg.norm(z_axis)
    y_axis = np.cross(z_axis, x_axis)
    y_axis /= np.linalg.norm(y_axis)
    x_axis = np.cross(y_axis, z_axis)
    rotation = np.column_stack((x_axis, y_axis, z_axis))
    return Rotation.from_matrix(rotation).as_quat()


def make_grasp_candidates(box: OrientedBox, grasp_depth, tcp_to_grasp_xyz):
    # Piper-X's finger motion resolves to TCP X in the MoveIt URDF. Keep that
    # closing axis on the object's short edge; the second pose reverses it 180 deg.
    z_tcp = np.array([0.0, 0.0, -1.0])
    grasp_center = np.array([box.center[0], box.center[1], box.top_z - grasp_depth])
    candidates = []
    for sign in (1.0, -1.0):
        quaternion = quaternion_from_axes(sign * box.short_axis, z_tcp)
        rotation = Rotation.from_quat(quaternion).as_matrix()
        tcp_position = grasp_center - rotation @ np.asarray(tcp_to_grasp_xyz)
        candidates.append((tcp_position, quaternion))
    return candidates
