from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from std_msgs.msg import Bool

from .api import Api


class MagnetApi(Api):
    def __init__(self, node: Node, drone_id : int):

        pass
