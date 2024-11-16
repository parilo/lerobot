import time
import torch as t
import numpy as np
from lerobot.common.robot2.util import read_json_file
from lerobot.common.robot_devices.motors.feetech import FeetechMotorsBus
from feetech_robot import FeetechRobot  # Adjust import path as needed


def main():

    np.set_printoptions(suppress=True, precision=4)
    t.set_printoptions(sci_mode=False, precision=4)

    # Initialize FeetechMotorsBus with specific motor configuration
    servos = FeetechMotorsBus(
        port='/dev/tty.usbmodem58CD1772711',
        motors={
            # name: (index, model)
            'shoulder_pan': [1, "sts3215"],
            'shoulder_lift': [2, "sts3215"],
            'elbow_flex': [3, "sts3215"],
            'wrist_flex': [4, "sts3215"],
            'wrist_roll': [5, "sts3215"],
            'gripper': [6, "sts3215"],
        }
    )

    motors_calibration_path = '.cache/calibration/so100/main_follower.json'
    servos.set_calibration(read_json_file(motors_calibration_path))

    # Initialize the robot with the servos
    robot = FeetechRobot(motors=servos)

    try:
        # Connect to the robot
        robot.connect()

        # Define relax position
        RELAX_POS = np.array([-0.3516, 195.6445, 182.3730, 72.6855, -0.3516, 0.8257])

        # Move to relax position
        robot.position_control(RELAX_POS)
        time.sleep(1)

        # Read current observation
        observation = robot.read()
        print(f'--- obs positions: {observation.pos}, velocities: {observation.vel}')

        # Set up sinusoidal movement

        num_steps = 300
        freq = 25

        for ind in range(num_steps):
            # Update target positions with sinusoidal motion
            target_pos = RELAX_POS.copy()
            target_pos[5] = 45 - 45 * t.cos(t.tensor(2 * ind / freq))
            target_pos[3] = 36 + 36 * t.cos(t.tensor(2 * ind / freq))
            target_pos[2] = 151 + 31 * t.cos(t.tensor(2 * ind / freq))
            target_pos[1] = 170 + 25 * t.cos(t.tensor(2 * ind / freq))

            print(target_pos)
            robot.position_control(target_pos)
            time.sleep(1 / freq)

        # Move back to relax position
        robot.position_control(RELAX_POS)

    finally:
        robot.relax()
        # Disconnect the robot
        robot.disconnect()


if __name__ == "__main__":
    main()
