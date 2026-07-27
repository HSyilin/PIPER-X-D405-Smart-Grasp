import numpy as np

from smart_grasp.depth_geometry import (
    depth_to_meters,
    estimate_oriented_box,
    make_grasp_candidates,
    masked_points,
    size_matches,
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


def test_obb_size_and_grasp_candidates():
    rng = np.random.default_rng(3)
    points = np.column_stack((
        rng.uniform(-0.02, 0.02, 3000),
        rng.uniform(-0.03, 0.03, 3000),
        rng.uniform(0.038, 0.040, 3000),
    ))
    box = estimate_oriented_box(points, table_z=0.0)
    assert size_matches(box.size, [0.060, 0.040, 0.040], 0.004)
    candidates = make_grasp_candidates(box, 0.020, [0.0, 0.0, 0.1425])
    assert len(candidates) == 2
    assert all(position[2] > box.top_z for position, _ in candidates)
