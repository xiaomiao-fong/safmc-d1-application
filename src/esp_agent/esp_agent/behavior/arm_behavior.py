from typing import Optional

from .behavior import Behavior
from d1_api import (DroneApi, MediatorApi, MagnetApi, ArucoApi)
from utils.logger import Logger

from px4_msgs.msg import VehicleStatus

class ArmBehavior(Behavior):
    def __init__(self, logger: Logger, drone_api: DroneApi, mediator_api: MediatorApi):
        super().__init__(logger)
        self.drone_api = drone_api
        self.mediator_api = mediator_api

    def execute(self):

        if not self.drone_api.is_armed:
            self.drone_api.arm()

    def get_next_state(self) -> Optional[str]:
        if self.mediator_api.received_teleop_signal:
            return "teleop"
        return None