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

        self.__clock: Clock = node.get_clock()

        # Initial Values
        self.__takeoff_state = False
        self.__agent_state = -1
        self.__control_state = -1
        self.node = node
        self.drone_id = drone_id
        self.topic_prefix = f"/drone_{self.drone_id}"
        
        self.__arming_signal : bool = False

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
        # Subscribers

        self.node.create_subscription(Bool, f"{self.topic_prefix}/in/arm", self.__set_arming_signal, qos_profile)

        #Publishers

        self.arm_ready_pub = self.node.create_publisher(Bool, f"{self.topic_prefix}/out/arm_ready", qos_profile)


    # Properties

    @property
    def received_arming_signal(self) -> bool:
        return self.__arming_signal
        
    # Setters

    
    def __set_status(self, msg: State):
        self.__takeoff_state = msg.takeoff_state
        self.__agent_state = msg.agent_state
        self.__control_state = msg.control_state

    def __set_is_ok_to_arm(self, msg: Bool):
        self.__is_ok_to_arm = msg.data
    def __set_arming_signal(self, msg : Bool) -> None:
        self.__arming_signal = msg.data

    # API for Mediator

    def online(self) -> None:
        online_msg = Bool()
        online_msg.data = True
        self.arm_ready_pub.publish(online_msg)


