#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class Grab(Node):
    def __init__(self):
        super().__init__("frame_grabber")
        self.bridge = CvBridge()
        self.got = {}
        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, "/camera/camera/color/image_raw",
                                 lambda m: self.save(m, "color"), qos)
        self.create_subscription(Image, "/smart_grasp/debug_image",
                                 lambda m: self.save(m, "debug"), qos)

    def save(self, msg, name):
        if name in self.got:
            return
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imwrite(f"/tmp/frame_{name}.png", img)
        self.got[name] = True
        print(f"saved {name} {img.shape} encoding={msg.encoding}")
        if len(self.got) == 2:
            raise SystemExit

rclpy.init()
n = Grab()
try:
    rclpy.spin(n)
except SystemExit:
    pass
