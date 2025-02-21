from typing import Optional

from .behavior import Behavior
from d1_api import (DroneApi, MediatorApi, MagnetApi, ArucoApi)
from utils.logger import Logger

from px4_msgs.msg import VehicleStatus

class IdleBehavior(Behavior):
    def __init__(self, logger: Logger, drone_api: DroneApi, mediator_api: MediatorApi):
        super().__init__(logger)
        self.drone_api = drone_api
        self.mediator_api = mediator_api

    # def on_enter(self):
    #     self.mediator_api.reset_states()

    def execute(self):
        if self.drone_api.is_armed:
            self.drone_api.disarm()

        # self.drone_api.reset_start_position()

        pf_pass = self.drone_api.is_each_pre_flight_check_passed
        self.logger.info(f"Preflight checks passed: {pf_pass}")

        if (not self.drone_api.is_armed) and \
                pf_pass and \
                self.drone_api.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            
            self.mediator_api.online()

    def get_next_state(self) -> Optional[str]:
        if self.mediator_api.received_arming_signal:
            return "arm"
        return None