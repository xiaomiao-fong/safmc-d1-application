from typing import Optional

from .behavior import Behavior
from d1_api import (DroneApi, MediatorApi, MagnetApi, ArucoApi)
from utils.logger import Logger

from px4_msgs.msg import VehicleStatus

class LoadBehavior(Behavior):
    def __init__(self, logger: Logger, drone_api: DroneApi, mediator_api: MediatorApi, magnet_api: MagnetApi):
        super().__init__(logger)
        self.drone_api = drone_api
        self.mediator_api = mediator_api
        self.magnet_api = magnet_api

    def on_enter(self):
        self.magnet_api.activate_magnet()
        self.drone_api.disarm()

    def execute(self):

        # TODO load check

        # self.mediator_api.send_status("load",self.drone_api.local_position)
        self.magnet_api.activate_magnet()
        self.mediator_api.send_is_loaded()


    def get_next_state(self) -> Optional[str]:
        if self.mediator_api.received_arming_signal:
            return "arm"
        return None