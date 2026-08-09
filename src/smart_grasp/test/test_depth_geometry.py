import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from smart_grasp.depth_geometry import (
    depth_to_meters,
    estimate_oriented_box,
    make_grasp_candidates,
    masked_points,
    robust_filter,
)


def test_depth_units_and_projection():
    depth = np.full((3, 3), 200, dtype=np.uint16)
    mask = np.zeros((3, 3), dtype=np.uint8)
    mask[1, 2] = 255
    camera = np.array([[100.0, 0.0, 1.0], [0.0, 100.0, 1.0], [0.0, 0.0, 1.0]])
    points, ratio = masked_points(depth_to_meters(depth, "16UC1"), mask, camera, 0.08, 0.60)
    assert ratio == 1.0
    assert np.allclose(points[0], [0.002, 0.0, 0.2])
    assert np.allclose(depth_to_meters(depth.astype(np.float32) / 1000, "32FC1"), 0.2)


def test_fixed_geometry_and_grasp_candidates():
    rng = np.random.default_rng(3)
    points = np.column_stack((
        # Deliberately disagree with the configured dimensions. The point cloud
        # may determine pose, but it must never determine or reject target size.
        rng.uniform(-0.03, 0.03, 3000),
        rng.uniform(-0.03, 0.03, 3000),
        rng.uniform(0.038, 0.040, 3000),
    ))
    box = estimate_oriented_box(points, [0.060, 0.060, 0.040], table_z=0.0)
    assert np.allclose(box.size, [0.060, 0.060, 0.040])
    assert np.isclose(box.center[2] + 0.5 * box.size[2], box.top_z)
    candidates = make_grasp_candidates(box, 0.020, [0.0, 0.0, 0.1425])
    assert len(candidates) == 2
    assert all(position[2] > box.top_z for position, _ in candidates)
    candidate_rotations = [Rotation.from_quat(quaternion).as_matrix()
                           for _, quaternion in candidates]
    assert np.allclose(candidate_rotations[0][:, 0], box.short_axis)
    assert np.allclose(candidate_rotations[1][:, 0], -box.short_axis)


def test_outlier_radius_does_not_clip_large_configured_target():
    rng = np.random.default_rng(8)
    points = np.column_stack((
        rng.uniform(-0.05325, 0.05325, 4000),
        rng.uniform(-0.03825, 0.03825, 4000),
        rng.uniform(0.028, 0.030, 4000),
    ))
    clipped = robust_filter(points, radius=0.030)
    preserved = robust_filter(points, radius=0.080)

    assert np.ptp(clipped[:, 0]) < 0.070
    assert np.ptp(preserved[:, 0]) > 0.100
    assert np.ptp(preserved[:, 1]) > 0.070


def test_orientation_uses_top_surface_instead_of_blue_side_faces():
    rng = np.random.default_rng(11)
    top = np.column_stack((
        rng.uniform(-0.02, 0.02, 2500),
        rng.uniform(-0.053, 0.053, 2500),
        rng.uniform(0.039, 0.040, 2500),
    ))
    side = np.column_stack((
        rng.uniform(-0.060, 0.060, 5000),
        rng.uniform(-0.010, 0.010, 5000),
        rng.uniform(0.005, 0.030, 5000),
    ))

    box = estimate_oriented_box(
        np.vstack((top, side)), [0.060, 0.060, 0.040],
        table_z=0.010, orientation_surface_band=0.008,
    )
    assert abs(np.dot(box.short_axis, [1.0, 0.0, 0.0])) > 0.95


def test_minimum_area_rectangle_resists_uneven_surface_density():
    rng = np.random.default_rng(21)
    angle = np.deg2rad(30.0)
    rotation = np.array([
        [np.cos(angle), -np.sin(angle)],
        [np.sin(angle), np.cos(angle)],
    ])
    boundary = np.vstack((
        np.column_stack((np.full(200, -0.038), np.linspace(-0.053, 0.053, 200))),
        np.column_stack((np.full(200, 0.038), np.linspace(-0.053, 0.053, 200))),
        np.column_stack((np.linspace(-0.038, 0.038, 200), np.full(200, -0.053))),
        np.column_stack((np.linspace(-0.038, 0.038, 200), np.full(200, 0.053))),
    ))
    dense_patch = np.column_stack((
        rng.uniform(-0.035, -0.005, 3000),
        rng.uniform(-0.050, -0.020, 3000),
    ))
    xy = np.vstack((boundary, dense_patch)) @ rotation.T
    points = np.column_stack((xy, rng.uniform(0.039, 0.040, len(xy))))

    box = estimate_oriented_box(
        points, [0.060, 0.060, 0.040],
        table_z=0.010, orientation_surface_band=0.020,
    )

    expected_short_axis = rotation @ np.array([1.0, 0.0])
    assert abs(np.dot(box.short_axis[:2], expected_short_axis)) > 0.99
    assert np.linalg.norm(box.center[:2]) < 0.002


def test_degenerate_top_surface_is_rejected():
    points = np.column_stack((
        np.linspace(-0.05, 0.05, 20),
        np.zeros(20),
        np.full(20, 0.04),
    ))

    with pytest.raises(ValueError, match="degenerate top-surface rectangle"):
        estimate_oriented_box(points, [0.060, 0.040, 0.040], table_z=0.0)
