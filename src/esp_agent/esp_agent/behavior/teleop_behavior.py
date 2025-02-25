from typing import Optional

from .behavior import Behavior
from d1_api import (DroneApi, MediatorApi, MagnetApi, ArucoApi)
from utils.logger import Logger
from utils.coordinate import Coordinate

from px4_msgs.msg import VehicleStatus

class TeleopBehavior(Behavior):
    def __init__(self, logger: Logger, drone_api: DroneApi, mediator_api: MediatorApi):
        super().__init__(logger)
        self.drone_api = drone_api
        self.mediator_api = mediator_api

    # def on_enter(self):
    #     self.mediator_api.reset_states()

    def execute(self):
        
        self.drone_api.move_with_velocity(Coordinate(
            self.drone_api.espcmd.vx/2,
            self.drone_api.espcmd.vy/2,
            self.drone_api.espcmd.vz/2
        ), self.drone_api.espcmd.yaw)


    def get_next_state(self) -> Optional[str]:
        if self.mediator_api.received_drop_signal:
            return "drop"
        if self.mediator_api.received_load_signal:
            return "load"
        return None