import numpy as np
from scipy.spatial.transform import Rotation

from smart_grasp.stability import sample_spans


def test_sample_spans_reports_position_and_rotation_range():
    samples = [
        ([0.3, 0.0, 0.1], Rotation.from_euler("z", 0.0).as_quat()),
        ([0.31, 0.0, 0.1], Rotation.from_euler("z", 2.0, degrees=True).as_quat()),
    ]
    position, angle = sample_spans(samples)
    assert np.isclose(position, 0.01)
    assert np.isclose(angle, 2.0)
