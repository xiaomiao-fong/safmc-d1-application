# TODO 之後可以用 timestamp 的方式去偵測是否長時間沒收到 aruco marker 的資訊

from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from rclpy.time import Time

from .api import Api


class ArucoApi(Api):
    def __init__(self, node: Node):

        pass
