import rclpy
from rclpy.node import Node
from utils.coordinate import Coordinate
from utils.logger import Logger
from utils.parameters import get_parameter

from d1_api import (DroneApi, MediatorApi, MagnetApi, ArucoApi)

from .constant import DELTA_TIME
from esp_agent.agent_machine import AgentMachine

class Agent(Node):
    def __init__(self):
        super().__init__('agent')

        self.logger = Logger(self.get_logger(), self.get_clock())

        drone_id = get_parameter(self, 'drone_id', 1)

        self.logger.info(f"New agent: drone_id={drone_id}")

        self.drone_api = DroneApi(self, drone_id)
        self.mediator_api = MediatorApi(self, drone_id)
        self.magnet_api = MagnetApi(self, drone_id)
        self.aruco_api = ArucoApi(self)

        # State Machine
        self.machine = AgentMachine(
            self.logger, self.drone_api, self.magnet_api, self.mediator_api, self.aruco_api)

        self.timer = self.create_timer(DELTA_TIME, self.update)

    def update(self):
        # 要 2 Hz 發送, 否則會退出 offboard control mode
        self.drone_api.set_offboard_control_mode()

        # 傳送 agent status 給 mediator
        # self.mediator_api.send_status(self.machine.state.value, self.drone_api.local_position)

        self.machine.proceed()
        self.machine.execute()


def main(args=None):

    rclpy.init(args=args)
    agent = Agent()

    try:
        rclpy.spin(agent)
    finally:
        agent.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()