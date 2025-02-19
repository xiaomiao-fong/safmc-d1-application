import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

import serial
import threading
import json
import time

from .NEDCoordinate import NEDCoordinate
from esp_msg.msg import ESPCMD
from px4_msgs.msg import (GotoSetpoint, OffboardControlMode,
                          TrajectorySetpoint, VehicleCommand,
                          VehicleLocalPosition, VehicleStatus)


class Agent(Node):

    def __init__(self):
        super().__init__("Agent")
        self.get_logger().info("Starting task...")
        self.state = "INIT"
        self.drone_id = 1

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Initial Values
        self.is_armed = False
        self.vehicle_timestamp = 1
        self.is_each_pre_flight_check_passed = False
        self.start_position = NEDCoordinate(0, 0, 0)
        self.local_position = NEDCoordinate(0, 0, 0)
        self.heading = 0.0

        self.espcmd : ESPCMD = ESPCMD()
        # Subscriptions
        print(f"/fmu/out/vehicle_local_position")

        self.esp_vel_sub : rclpy.publisher.Publisher = self.create_subscription(ESPCMD, "/esp_vel", self.esp_cmd_callback ,qos_profile)
        
        self.vehicle_local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            f"/fmu/out/vehicle_local_position",
            self.__set_vehicle_local_position,
            qos_profile)

        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            f"/fmu/out/vehicle_status",
            self.__set_vehicle_status,
            qos_profile
        )

        # Publishers
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            f"/fmu/in/vehicle_command",
            qos_profile
        )

        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode,
            f"/fmu/in/offboard_control_mode",
            qos_profile
        )

        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            f"/fmu/in/trajectory_setpoint",
            qos_profile
        )

        self.create_timer(0.01, self.execute_state)
        self.create_timer(0.01, self.set_offboard_control_mode)
        
    def esp_cmd_callback(self, msg : ESPCMD):
        self.espcmd = msg

    def execute_state(self):
        match self.state:
            case "INIT":

                self.get_logger().info(f"Armed status: {self.is_armed}")
                self.get_logger().info(f"Vehicle timestamp: {self.vehicle_timestamp}")
                self.get_logger().info(f"Preflight checks passed: {self.is_each_pre_flight_check_passed}")

                if (not self.is_armed) and self.is_each_pre_flight_check_passed:
                    self.get_logger().info("Drone is ready to arm and start offboard control.")
                    self.activate_offboard_control_mode()
                    self.arm()
                    self.get_logger().info("Ok")
                    self.state = "TELEOP"
                    

            case "TELEOP":
                self.move_with_velocity()

    
    def move_with_velocity(self):
        trajectory_setpoint_msg = TrajectorySetpoint()
        trajectory_setpoint_msg.timestamp = self.vehicle_timestamp

        trajectory_setpoint_msg.velocity[0] = 0.0
        trajectory_setpoint_msg.velocity[1] = self.espcmd.vy
        trajectory_setpoint_msg.velocity[2] = self.espcmd.vz
        trajectory_setpoint_msg.yawspeed = 0.0

        trajectory_setpoint_msg.position[0] = 0.0
        trajectory_setpoint_msg.position[1] = 0.0
        trajectory_setpoint_msg.position[2] = 0.0
        trajectory_setpoint_msg.yaw = self.heading

        self.trajectory_setpoint_pub.publish(trajectory_setpoint_msg)

    def __set_vehicle_status(self, vehicle_status_msg: VehicleStatus) -> None:
        self.is_each_pre_flight_check_passed = vehicle_status_msg.pre_flight_checks_pass
        self.vehicle_timestamp = vehicle_status_msg.timestamp
        self.is_armed = (
            vehicle_status_msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        )        

    def __set_vehicle_local_position(self, vehicle_local_position_msg: VehicleLocalPosition):
        self.heading = vehicle_local_position_msg.heading
        self.local_position = NEDCoordinate(
            x=vehicle_local_position_msg.x,
            y=vehicle_local_position_msg.y,
            z=vehicle_local_position_msg.z
        )

    def __get_default_vehicle_command_msg(self, command, *params: float, **kwargs):
        '''
        Generate the vehicle command.\n
        defaults:\n
            params[0:7] = 0
            target_system = self.drone_id - 1\n
            target_component = 1\n
            source_system = self.drone_id - 1\n
            source_component = 1\n
            from_external = True\n
            timestamp = int(timestamp / 1000)
        '''
        vehicle_command_msg = VehicleCommand()
        vehicle_command_msg.timestamp = self.vehicle_timestamp

        # command
        vehicle_command_msg.command = command

        # params
        params = list(params) + [0] * (7 - len(params))
        for i, param in enumerate(params[:7], start=1):
            setattr(vehicle_command_msg, f'param{i}', float(param))

        # defaults
        vehicle_command_msg.target_system = self.drone_id - 1
        vehicle_command_msg.target_component = 0 # all components
        vehicle_command_msg.source_system = self.drone_id - 1
        vehicle_command_msg.source_component = 0 # all components
        vehicle_command_msg.from_external = True

        # other kwargs
        for attr, value in kwargs.items():
            try:
                setattr(vehicle_command_msg, attr, value)
            except Exception as e:
                print(e)

        return vehicle_command_msg
    
    def arm(self) -> None:
        """
        Arms the drone for flight.

        Sends a command to the vehicle to arm it, allowing flight to proceed.
        This command uses `VEHICLE_CMD_COMPONENT_ARM_DISARM` with `param1=1` to arm the vehicle.
        """

        vehicle_command_msg = self.__get_default_vehicle_command_msg(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            1
        )

        self.vehicle_command_pub.publish(vehicle_command_msg)

    def set_offboard_control_mode(self) -> None:
        """
        Set the offboard control mode for the drone.

        position = True
        velocity = True
        timestamp = current timestamp

        ref: https://docs.px4.io/main/en/flight_modes/offboard.html#ros-2-messages
        """
        offboard_control_mode_msg = OffboardControlMode()
        offboard_control_mode_msg.timestamp = self.vehicle_timestamp
        offboard_control_mode_msg.position = True  # TrajectorySetpoint
        self.offboard_control_mode_pub.publish(offboard_control_mode_msg)

    def activate_offboard_control_mode(self) -> None:
        """
        Activates the offboard control mode for the drone.

        This method sends a `VehicleCommand` message to switch the drone to offboard mode 
        by setting the appropriate control mode flags. The mode is switched by using the
        `VEHICLE_CMD_DO_SET_MODE` command.
        """
        vehicle_command_msg = self.__get_default_vehicle_command_msg(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            1,
            6
        )

        self.vehicle_command_pub.publish(vehicle_command_msg)

def main(args = None):
    rclpy.init(args=args)
    node = Agent()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()