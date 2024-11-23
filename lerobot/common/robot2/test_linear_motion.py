import time
import numpy as np
from lerobot.common.robot2.joint_traj import generate_joint_trajectory, play_joint_trajectory
from lerobot.common.robot2.robot import Robot
from lerobot.common.robot2.virtual_robot import VirtualRobot
from vrteleop.ik import Kinematics


def main():

    np.set_printoptions(suppress=True, precision=2)
    # t.set_printoptions(sci_mode=False, precision=4)

    # Initialize the robot with the servos
    urdf_path = '/Users/parilo/devel/armlib/SO-ARM100/URDF/SO_5DOF_ARM100_8j_URDF.SLDASM/urdf/SO_5DOF_ARM100_8j_URDF.SLDASM_2.urdf'

    kinematics = Kinematics(
        urdf_path=urdf_path,
        end_link_name="Moving Jaw",
    )

    robot = VirtualRobot(
        urdf_path=urdf_path,
        kinematics=kinematics,
        end_link_name="Moving Jaw",
    )

    try:
        # Connect to the robot
        robot.connect()

        # Define relax position
        RELAX_POS = np.array([0., 195, 182, 72.6855, 0, 0])
        START_POS = np.array([0., 143, 129, 72.6855, 0, 0])

        # ik.jac(START_POS, np.array([0, 0, 0.01, 0, 0, 0]))

        traj = generate_joint_trajectory(
            start_pos=RELAX_POS,
            end_pos=START_POS,
            max_joint_vel=90,
        )
        play_joint_trajectory(
            robot=robot,
            trajectory=traj,
        )

        num_steps = 1000
        freq = 25
        dir = 1

        for ind in range(num_steps):
            # Read current observation
            observation = robot.read()
            cur_joints = observation.pos

            coord = kinematics.fk(np.deg2rad(cur_joints))['Moving Jaw'][1, 3]
            if coord > 0.1:
                dir = -1
            if coord < -0.1:
                dir = 1

            djoints = np.rad2deg(kinematics.djoints(
                js=np.deg2rad(cur_joints),
                dx=np.array([0, dir * 0.001, 0, 0, 0, 0]),
            ))
            robot.position_control(cur_joints + djoints)
            time.sleep(1 / freq)

        # Move back to relax position
        robot.position_control(RELAX_POS)

    finally:
        robot.relax()
        # Disconnect the robot
        robot.disconnect()


if __name__ == "__main__":
    main()
