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
        self.start_position = None
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
        self.get_logger().info(f"{self.state}")
        match self.state:
            case "INIT":
                self.get_logger().info(
                    f"Armed: {self.is_armed}, Timestamp: {self.vehicle_timestamp}, Preflight: {self.is_each_pre_flight_check_passed}"
                )
                if self.is_each_pre_flight_check_passed:
                    if self.start_position is None:
                        self.start_position = NEDCoordinate(
                            self.local_position.x, self.local_position.y, self.local_position.z
                        )
                        self.get_logger().info(f"Recorded start position: {self.start_position}")
                    self.activate_offboard_control_mode()
                    self.arm()
                    self.get_logger().info("Armed and initiating takeoff...")
                    self.state = "TAKEOFF"

            case "TAKEOFF":
                TAKEOFF_HEIGHT = 0.5  # meters
                target_z = self.start_position.z - TAKEOFF_HEIGHT
                # Calculate altitude error (in meters)
                error = self.local_position.z - target_z
                tolerance = 0.05  # acceptable error in meters
                self.get_logger().info(f"{error}")

                # Publish a takeoff setpoint with a controlled descent velocity.
                self.perform_takeoff()

                if abs(error) <= tolerance:
                    self.get_logger().info("Reached target altitude. Transitioning to HOLD state.")
                    self.state = "HOLD"

            case "HOLD":
                self.get_logger().info(f"{self.state}")
                self.hold()
                
    def hold(self) -> None:
        trajectory_setpoint_msg = TrajectorySetpoint()
        trajectory_setpoint_msg.timestamp = self.vehicle_timestamp

        trajectory_setpoint_msg.position[0] = float("nan")
        trajectory_setpoint_msg.position[1] = float("nan")
        trajectory_setpoint_msg.position[2] = float("nan")

        trajectory_setpoint_msg.velocity[0] = 0.0
        trajectory_setpoint_msg.velocity[1] = 0.0
        trajectory_setpoint_msg.velocity[2] = 0.0

        trajectory_setpoint_msg.yaw = self.heading

        self.trajectory_setpoint_pub.publish(trajectory_setpoint_msg)

    def perform_takeoff(self) -> None:

        TAKEOFF_HEIGHT = 0.5  # meters
        target_z = self.start_position.z - TAKEOFF_HEIGHT

        error = self.local_position.z - target_z

        Kp = -1.2
        desired_velocity = Kp * error  
        max_vel = 0.6
        if desired_velocity > max_vel:
            desired_velocity = max_vel
        elif desired_velocity < -max_vel:
            desired_velocity = -max_vel

        if abs(error) < 0.05:
            desired_velocity = 0.0
            
        self.get_logger().info(f"{desired_velocity}")

        trajectory_setpoint_msg = TrajectorySetpoint()
        trajectory_setpoint_msg.timestamp = self.vehicle_timestamp

        trajectory_setpoint_msg.position[0] = float("nan")
        trajectory_setpoint_msg.position[1] = float("nan")
        trajectory_setpoint_msg.position[2] = float("nan")

        trajectory_setpoint_msg.velocity[0] = 0.0
        trajectory_setpoint_msg.velocity[1] = 0.0
        trajectory_setpoint_msg.velocity[2] = desired_velocity

        trajectory_setpoint_msg.yaw = self.heading

        self.trajectory_setpoint_pub.publish(trajectory_setpoint_msg)
    
    def move_with_velocity(self):
        trajectory_setpoint_msg = TrajectorySetpoint()
        trajectory_setpoint_msg.timestamp = self.vehicle_timestamp

        trajectory_setpoint_msg.velocity[0] = 0.02
        trajectory_setpoint_msg.velocity[1] = 0.0
        trajectory_setpoint_msg.velocity[2] = 0.0
        trajectory_setpoint_msg.yawspeed = 0.0

        trajectory_setpoint_msg.position[0] = float("nan")
        trajectory_setpoint_msg.position[1] = float("nan")
        trajectory_setpoint_msg.position[2] = float("nan")
        trajectory_setpoint_msg.yaw = self.heading

        self.trajectory_setpoint_pub.publish(trajectory_setpoint_msg)

    def __set_vehicle_status(self, vehicle_status_msg: VehicleStatus) -> None:
        self.is_each_pre_flight_check_passed = vehicle_status_msg.pre_flight_checks_pass
        self.vehicle_timestamp = vehicle_status_msg.timestamp
        self.nav_state = vehicle_status_msg.nav_state
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
        vehicle_command_msg.target_system = 0
        vehicle_command_msg.target_component = 0 # all components
        vehicle_command_msg.source_system = 0
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
        offboard_control_mode_msg.velocity = True  # TrajectorySetpoint
        offboard_control_mode_msg.position = False
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


    # def perform_takeoff(self) -> None:

    #     vehicle_command_msg = self.__get_default_vehicle_command_msg(
    #         VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
    #         1,
    #         4,
    #         2
    #     )
    #     self.vehicle_command_pub.publish(vehicle_command_msg)

def main(args = None):
    rclpy.init(args=args)
    node = Agent()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()