import time
import numpy as np
import torch as t
import argparse
from lerobot.common.robot2.feetech_robot import FeetechRobot
from lerobot.common.robot2.util import read_json_file, trim_repeated_values
from lerobot.common.robot_devices.motors.feetech import FeetechMotorsBus  # Assuming your class is defined in feetech_robot.py


def main():

    np.set_printoptions(suppress=True, precision=4)
    t.set_printoptions(sci_mode=False, precision=4)

    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Capture robot trajectory and save it to a file.")
    parser.add_argument(
        '--traj-path', '-t',
        type=str,
        required=True,
        help="Path where the trajectory data should be saved (e.g., 'robot_trajectory.npy')."
    )
    parser.add_argument(
        '--freq',
        type=float,
        default=25.,
        required=False,
        help="Frequency (in Hz) at which to capture the robot's trajectory. E.g., 10.0 means 10 samples per second."
    )
    args = parser.parse_args()

    # Initialize the robot
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
    robot.connect()

    # Define relax position
    RELAX_POS = np.array([-0.3516, 195.6445, 182.3730, 72.6855, -0.3516, 0.8257])

    # Move to relax position
    robot.position_control(RELAX_POS)
    time.sleep(1)
    robot.relax()

    trajectory = []

    print(f"Capturing trajectory at {args.freq} Hz... Press Ctrl+C to stop.")
    try:
        while True:
            # Capture observation from the robot
            t1 = time.time()
            observation = robot.read().pos
            t2 = time.time()
            trajectory.append(observation)  # Save the current state

            print(f"Captured: {observation} dt {(t2 - t1) * 1000}")
            time.sleep(1 / args.freq)  # Adjust the sampling interval to match the specified frequency

    except KeyboardInterrupt:
        print("Trajectory capture stopped by user.")

    finally:
        # Save trajectory to the specified file path
        trajectory_array = np.array(trajectory)
        print(f'--- traj shape 1 {trajectory_array.shape}')
        trajectory_array = trim_repeated_values(trajectory_array)
        print(f'--- traj shape 2 {trajectory_array.shape}')
        np.save(args.traj_path, trajectory_array)
        print(f"Trajectory saved to {args.traj_path}")

        # Disconnect the robot
        robot.relax()
        robot.disconnect()
        print("Robot disconnected.")


if __name__ == "__main__":
    main()
