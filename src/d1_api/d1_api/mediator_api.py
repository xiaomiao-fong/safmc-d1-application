from typing import List, Optional

from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from std_msgs.msg import Bool, UInt32

from common.coordinate import Coordinate

from esp_msg.msg import State

from .api import Api


class MediatorApi(Api):
    def __init__(self, node: Node, drone_id: int):

        self.__drone_id = drone_id

        self.__clock: Clock = node.get_clock()

        # Initial Values
        self.__takeoff_state = False
        self.__agent_state = -1
        self.__control_state = -1

        self.__supply_zone: List[Optional[Coordinate]] = [None, None, None, None]
        self.__drop_zone: Optional[Coordinate] = None

        self.__obstacle_array: List[Coordinate] = []  # list

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriptions
        node.create_subscription(State, "/agent/state", self.__set_status, qos_profile)
        node.create_subscription(Bool, f"/agent_{self.__drone_id}/cmd_arm", self.__set_is_ok_to_arm, qos_profile)

        # Publishers

    @property
    def takeoff_state(self):
        return self.__takeoff_state
    @property
    def agent_state(self):
        return self.__agent_state
    @property
    def control_state(self):
        return self.__control_state
    @property
    def is_ok_to_arm(self):
        return self.__is_ok_to_arm

    
    def __set_status(self, msg: State):
        self.__takeoff_state = msg.takeoff_state
        self.__agent_state = msg.agent_state
        self.__control_state = msg.control_state

    def __set_is_ok_to_arm(self, msg: Bool):
        self.__is_ok_to_arm = msg.data