from typing import Optional

from .behavior import Behavior
from d1_api import (DroneApi, MediatorApi, MagnetApi, ArucoApi)
from utils.logger import Logger
from utils.coordinate import Coordinate

from px4_msgs.msg import VehicleStatus

class HoldBehavior(Behavior):
    def __init__(self, logger: Logger, drone_api: DroneApi, mediator_api: MediatorApi):
        super().__init__(logger)
        self.drone_api = drone_api
        self.mediator_api = mediator_api

    def on_enter(self):
        self.mediator_api.set_hold_signal(False)
        self.target_position = self.drone_api.local_position 

    def execute(self):
        self.mediator_api.send_status()
        self.drone_api.move_with_velocity(Coordinate(0,0,0), 0)

        # TODO hold method

    def get_next_state(self) -> Optional[str]:
        if self.mediator_api.received_teleop_signal:
            return "teleop"
        if self.mediator_api.received_drop_signal:
            return "drop"
        if self.mediator_api.received_track_signal:
            return "track"
        return None