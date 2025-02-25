from typing import List, Optional

from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from std_msgs.msg import Bool, UInt32
from esp_msg.msg import AgentStatus
from common.coordinate import Coordinate

from .api import Api


class MediatorApi(Api):
    def __init__(self, node: Node, drone_id: int):

        self.node = node
        self.__drone_id = drone_id
        self.__topic_prefix = f"/drone_{self.__drone_id}"
        
        self.__arming_signal : bool = False
        self.__teleop_signal : bool = False
        self.__load_signal : bool = False
        self.__drop_signal : bool = False

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers

        node.create_subscription(Bool, f"{self.__topic_prefix}/arm", self.__set_arming_signal, qos_profile)
        node.create_subscription(Bool, f"{self.__topic_prefix}/teleop", self.__set_teleop_signal, qos_profile)
        node.create_subscription(Bool, f"{self.__topic_prefix}/load", self.__set_load_signal, qos_profile)

        # TODO mediator outgoing msg
        node.create_subscription(Bool, f"{self.__topic_prefix}/drop", self.__set_drop_signal, qos_profile)
        node.create_subscription(Bool, f"{self.__topic_prefix}/track", self.__set_track_signal, qos_profile)

        #Publishers

        self.arm_ready_pub = node.create_publisher(UInt32, f"/mediator/arm_ready", qos_profile)
        self.status_pub = node.create_publisher(AgentStatus, '/mediator/status', qos_profile)
        self.loaded_pub = node.create_publisher(UInt32, '/mediator/loaded', qos_profile)


    ### Properties ###

    @property
    def received_arming_signal(self) -> bool:
        return self.__arming_signal
    @property
    def received_teleop_signal(self) -> bool:
        return self.__teleop_signal
    @property
    def received_load_signal(self) -> bool:
        return self.__load_signal
    @property
    def received_drop_signal(self) -> bool:
        return self.__drop_signal
    @property
    def received_track_signal(self) -> bool:
        return self.__track_signal
        
    ### Setters ###

    def __set_arming_signal(self, msg : Bool) -> None:
        self.__arming_signal = msg.data

    def __set_teleop_signal(self, msg : Bool) -> None:
        self.__teleop_signal = msg.data

    def __set_load_signal(self, msg : Bool) -> None:
        self.__load_signal = msg.data

    def __set_drop_signal(self, msg : Bool) -> None:
        self.__drop_signal = msg.data
    
    def __set_track_signal(self, msg : Bool) -> None:
        self.__track_signal = msg.data

    def __get_drone_id_msg(self) -> UInt32:
        uint32_msg = UInt32()
        uint32_msg.data = self.__drone_id
        return uint32_msg

    ### API for Mediator ###

    def online(self) -> None:
        self.arm_ready_pub.publish(self.__get_drone_id_msg())

    def send_status(self, state_name: str, local_position: Coordinate):
        if state_name is None or local_position is None:
            return
        agent_status_msg = AgentStatus()
        agent_status_msg.drone_id = self.__drone_id
        agent_status_msg.point.x = float(local_position.x)
        agent_status_msg.point.y = float(local_position.y)
        agent_status_msg.point.z = float(local_position.z)
        self.status_pub.publish(agent_status_msg)

    def send_is_loaded(self):
        self.loaded_pub.publish(self.__get_drone_id_msg())
        

