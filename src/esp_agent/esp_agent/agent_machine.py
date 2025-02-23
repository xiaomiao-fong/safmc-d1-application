from transitions import Machine
from enum import Enum

from d1_api import (DroneApi, MagnetApi, MediatorApi, ArucoApi)
from utils.logger import Logger

from esp_agent.behavior import (Behavior, IdleBehavior, ArmBehavior, HoldBehavior, LoadBehavior)

class States(Enum):
    IDLE = 0
    ARM = 1
    TELEOP = 2
    HOLD = 3
    LOAD = 4
    TRACK = 5
    DROP = 6

transitions = [
    {"source" : States.IDLE, "dest" : States.ARM},
    {"source" : States.ARM, "dest" : States.TELEOP},
    {"source" : States.TELEOP, "dest" : States.ARM},
    {"source" : States.TELEOP, "dest" : States.HOLD},
    {"source" : States.TELEOP, "dest" : States.LOAD},
    {"source" : States.TELEOP, "dest" : States.DROP},
    {"source" : States.HOLD, "dest" : States.TELEOP},
    {"source" : States.HOLD, "dest" : States.DROP},
    {"source" : States.HOLD, "dest" : States.TRACK},
    {"source" : States.LOAD, "dest" : States.ARM},
    {"source" : States.LOAD, "dest" : States.TELEOP},
    {"source" : States.TRACK, "dest" : States.ARM},
    {"source" : States.TRACK, "dest" : States.DROP},
    {"source" : States.DROP, "dest" : States.ARM}
]

class AgentMachine(Machine):
    
    def __init__(self, logger: Logger,
                 drone_api: DroneApi,
                 magnet_api: MagnetApi,
                 mediator_api: MediatorApi,
                 aruco_api: ArucoApi):

        self.logger = logger

        # behavior binding
        self.state_behavior_map = {
            States.IDLE: IdleBehavior(logger, drone_api, mediator_api),
            States.ARM: ArmBehavior(logger, drone_api, mediator_api),
            States.HOLD: ArmBehavior(logger, drone_api, mediator_api),
            States.LOAD: ArmBehavior(logger, drone_api, mediator_api, magnet_api)
        }

        # add state on_enter/on_exit callback
        states = [
            {
                'name': state_name,
                'on_enter': getattr(behavior, 'on_enter', None),
                'on_exit': getattr(behavior, 'on_exit', None)
            }
            for state_name, behavior in self.state_behavior_map.items()
        ]

        def populate_triggers(transitions):
            """
            Adds a trigger for each transition based on the destination state.

            The trigger name is the destination state converted to lowercase.

            For example:
            - States.TAKEOFF → "takeoff"
            - States.WALK_TO_SUPPLY → "walk_to_supply"
            """
            return [
                {**transition, 'trigger': transition['dest'].name.lower()}
                for transition in transitions
            ]

        super().__init__(self, states=states,
                         transitions=populate_triggers(transitions), initial=States.IDLE)
        
    def execute(self):
        """
        Executes the behavior of the current state.
        """
        self.logger.info(self.state.name)
        behavior: Behavior = self.state_behavior_map.get(self.state)
        if behavior:
            behavior.execute()

    def proceed(self):
        """
        Checks conditions and triggers state transitions.
        """
        behavior: Behavior = self.state_behavior_map.get(self.state)
        if behavior:
            if (next_state := behavior.get_next_state()):
                self.trigger(next_state)