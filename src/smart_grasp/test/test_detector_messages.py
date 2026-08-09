from sensor_msgs.msg import Image

from smart_grasp.detector_node import header_in_frame


def test_header_in_frame_does_not_mutate_source_message():
    source = Image()
    source.header.frame_id = "camera_color_optical_frame"

    copied = header_in_frame(source.header, "base_link")

    assert copied.frame_id == "base_link"
    assert source.header.frame_id == "camera_color_optical_frame"
