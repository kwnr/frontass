import rclpy
from rclpy.node import Node

import numpy as np

from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation

class FramePublisher(Node):
    def __init__(self):
        super().__init__('armstrong_tf2_frame_publisher')
        self.tf2_broadcaster = TransformBroadcaster(self)
