import time
import numpy as np
import argparse
from lerobot.common.robot2.feetech_robot import FeetechRobot
from lerobot.common.robot2.util import read_json_file
from lerobot.common.robot_devices.motors.feetech import FeetechMotorsBus  # Assuming your class is defined in feetech_robot.py


def main():
    np.set_printoptions(suppress=True, precision=4)

    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Play back a recorded robot trajectory.")
    parser.add_argument(
        '--traj-path', '-t',
        type=str,
        required=True,
        help="Path to the trajectory file (e.g., 'robot_trajectory.npy')."
    )
    parser.add_argument(
        '--freq',
        type=float,
        default=25.0,
        required=False,
        help="Playback frequency (in Hz). E.g., 10.0 means 10 samples per second."
    )
    args = parser.parse_args()

    # Load the trajectory from the file
    trajectory = np.load(args.traj_path)
    print(f"Loaded trajectory with shape: {trajectory.shape}")

    # Initialize the robot
    servos = FeetechMotorsBus(
        port='/dev/tty.usbmodem58CD1772711',
        motors={
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

    print(f"Playing back trajectory at {args.freq} Hz...")
    try:
        for idx, position in enumerate(trajectory):
            # Send each position to the robot
            robot.position_control(position)
            print(f"Played back {idx + 1}/{len(trajectory)}: {position}")
            time.sleep(1 / args.freq)  # Adjust the playback frequency

    except KeyboardInterrupt:
        print("Trajectory playback stopped by user.")

    finally:
        # Disconnect the robot
        robot.relax()
        robot.disconnect()
        print("Robot disconnected.")


if __name__ == "__main__":
    main()
