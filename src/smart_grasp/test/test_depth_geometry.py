import numpy as np

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
        rng.uniform(-0.04, 0.04, 3000),
        rng.uniform(-0.055, 0.055, 3000),
        rng.uniform(0.038, 0.040, 3000),
    ))
    box = estimate_oriented_box(points, [0.060, 0.040, 0.040], table_z=0.0)
    assert np.allclose(box.size, [0.060, 0.040, 0.040])
    candidates = make_grasp_candidates(box, 0.020, [0.0, 0.0, 0.1425])
    assert len(candidates) == 2
    assert all(position[2] > box.top_z for position, _ in candidates)


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
