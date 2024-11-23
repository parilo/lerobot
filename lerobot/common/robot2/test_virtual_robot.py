import time
import numpy as np
from lerobot.common.robot2.util import read_json_file
from lerobot.common.robot2.virtual_robot import VirtualRobot
from vrteleop.ik import IK
import torch as t


def main():

    np.set_printoptions(suppress=True, precision=4)
    t.set_printoptions(sci_mode=False, precision=4)

    # Initialize the robot with the servos
    urdf_path = '/Users/parilo/devel/lerobot/SO-ARM100/URDF/SO_5DOF_ARM100_8j_URDF.SLDASM/urdf/SO_5DOF_ARM100_8j_URDF.SLDASM_2.urdf'
    robot = VirtualRobot(
        urdf_path=urdf_path,
        kinematics=IK(
            urdf_path=urdf_path,
            end_link_name="Moving Jaw",
        ),
    )

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

        num_steps = 1000
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
            # robot.position_control(observation.pos)
            time.sleep(1 / freq)

        # Move back to relax position
        robot.position_control(RELAX_POS)

    finally:
        robot.relax()
        # Disconnect the robot
        robot.disconnect()


if __name__ == "__main__":
    main()
