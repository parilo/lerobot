import numpy as np
from dataclasses import dataclass
from lerobot.common.robot_devices.motors.feetech import FeetechMotorsBus


@dataclass
class MotorData:
    pos: np.ndarray
    vel: np.ndarray


class FeetechRobot:

    def __init__(self, motors: FeetechMotorsBus) -> None:
        self.motors = motors
        self.is_connected = False

    def connect(self) -> None:
        if not self.is_connected:
            self.motors.connect()
            self.is_connected = True
        else:
            print("Robot is already connected.")

    def disconnect(self) -> None:
        if self.is_connected:
            self.motors.disconnect()
            self.is_connected = False
        else:
            print("Robot is already disconnected.")

    def relax(self) -> None:
        """
        Relax the motors by disabling torque.
        """
        if not self.is_connected:
            raise RuntimeError("Robot is not connected.")
        self.motors.write("Torque_Enable", np.zeros(len(self.motors.motor_names), dtype=np.int32))

    def read(self) -> MotorData:
        """
        Read the current positions and velocities of all motors.
        """
        if not self.is_connected:
            raise RuntimeError("Robot is not connected.")

        positions = self.motors.read("Present_Position")
        velocities = self.motors.read("Present_Speed")

        return MotorData(pos=positions, vel=velocities)

    def position_control(self, target_pos: np.ndarray) -> None:
        """
        Control the motors to move to specific target positions.
        """
        if not self.is_connected:
            raise RuntimeError("Robot is not connected.")

        if len(target_pos) != len(self.motors.motor_names):
            raise ValueError("Target positions array length does not match the number of motors.")

        self.motors.write("Goal_Position", target_pos.astype(np.int32))
