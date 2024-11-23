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
    urdf_path = '/Users/parilo/devel/armlib/SO-ARM100/URDF/SO_5DOF_ARM100_8j_URDF.SLDASM/urdf/SO_5DOF_ARM100_8j_URDF.SLDASM.urdf'

    kinematics = Kinematics(
        urdf_path=urdf_path,
        end_link_name="Fixed_Jaw",
        mod_matrix=np.eye(6)[:5],
    )

    robot = VirtualRobot(
        urdf_path=urdf_path,
        kinematics=kinematics,
        end_link_name="Fixed_Jaw",
    )

    try:
        # Connect to the robot
        robot.connect()

        # Define relax position
        RELAX_POS = np.array([0., 195, 182, 72.6855, 0])
        START_POS = np.array([0., 143, 129, 72.6855, 0])
        # START_POS = np.array([ -0., 140.29, 98.63, 55.64, 0.])
        # START_POS = np.array([ -0., 140.29, 110, 55.64, 0.])

        traj = generate_joint_trajectory(
            start_pos=RELAX_POS,
            end_pos=START_POS,
            max_joint_vel=90,
        )
        play_joint_trajectory(
            robot=robot,
            trajectory=traj,
        )

        num_steps = 10000
        freq = 25

        for ind in range(num_steps):
            # Read current observation
            observation = robot.read()
            cur_joints = observation.pos

            # ampl = 2
            # vz =  0.001 * ampl * np.sin(2 / ampl * ind / freq)
            # vy =  0.001 * ampl * np.cos(2 / ampl * ind / freq)
            ampl = 2
            vel = 0.001 * 2
            if ampl * ind % 400 < 100:
                vz = 0
                vy = vel
            elif ampl * ind % 400 < 200:
                vz = vel
                vy = 0
            elif ampl * ind % 400 < 300:
                vz = 0
                vy = -vel
            elif ampl * ind % 400 < 400:
                vz = -vel
                vy = 0
            else:
                vz = 0
                vy = 0

            # djoints = np.rad2deg(kinematics.djoints3d(
            djoints = np.rad2deg(kinematics.djoints(
                js=np.deg2rad(cur_joints),
                dx=np.array([0, vy, vz, 0, 0, 0]),
            ))
            print(f'--- cur_joints {cur_joints}')
            # djoints = np.zeros_like(cur_joints)
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
