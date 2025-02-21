from typing import List, Optional

from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from std_msgs.msg import Bool, UInt32
from common.coordinate import Coordinate

from .api import Api


class MediatorApi(Api):
    def __init__(self, node: Node, drone_id: int):

        self.node = node
        self.drone_id = drone_id
        self.topic_prefix = f"/drone_{self.drone_id}"
        
        self.__arming_signal : bool = False
        self.__teleop_signal : bool = False

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers

        self.node.create_subscription(Bool, f"{self.topic_prefix}/in/arm", self.__set_arming_signal, qos_profile)
        self.node.create_subscription(Bool, f"{self.topic_prefix}/in/teleop", self.__set_teleop_signal, qos_profile)

        #Publishers

        self.arm_ready_pub = self.node.create_publisher(Bool, f"{self.topic_prefix}/out/arm_ready", qos_profile)


    # Properties

    @property
    def received_arming_signal(self) -> bool:
        return self.__arming_signal
    
    @property
    def received_teleop_signal(self) -> bool:
        return self.__teleop_signal
        
    # Setters

    def __set_arming_signal(self, msg : Bool) -> None:
        self.__arming_signal = msg.data

    def __set_teleop_signal(self, msg : Bool) -> None:
        self.__teleop_signal = msg.data

    # API for Mediator

    def online(self) -> None:
        online_msg = Bool()
        online_msg.data = True
        self.arm_ready_pub.publish(online_msg)


