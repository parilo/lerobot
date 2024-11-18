import time
import numpy as np
from lerobot.common.robot2.robot import Robot


def generate_joint_trajectory(
    start_pos: np.ndarray,
    end_pos: np.ndarray,
    max_joint_vel: float = 90,
    frequency: float = 25.,
) -> np.ndarray:
    """
    Generate a joint trajectory between two positions.

    Args:
        start_pos (np.ndarray): Starting joint positions (in degrees or radians).
        end_pos (np.ndarray): Target joint positions (in degrees or radians).
        max_joint_vel (float): Maximum joint velocity (per second).
        frequency (float): Sampling frequency (Hz).

    Returns:
        np.ndarray: A 2D array where each row is a set of joint positions at a time step.
    """
    # Compute the total movement for each joint
    delta = end_pos - start_pos

    # Calculate the time required for the largest movement based on max_joint_vel
    max_time = np.max(np.abs(delta) / max_joint_vel)

    # Determine the number of steps based on the frequency and max_time
    num_steps = int(np.ceil(max_time * frequency))

    # Generate a normalized time array
    t = np.linspace(0, 1, num_steps)

    # Interpolate positions for each joint
    trajectory = np.outer(t, delta) + start_pos

    return trajectory


def play_joint_trajectory(
        robot: Robot,
        trajectory: np.ndarray,
        freq: float = 25.0
        ) -> None:
    """
    Plays a joint trajectory on the virtual robot.

    Args:
        robot (VirtualRobot): The virtual robot to control.
        trajectory (np.ndarray): A 2D array where each row represents joint positions for one timestep.
        freq (float): Frequency in Hz at which to play the trajectory (default is 25 Hz).
    """
    num_steps, _ = trajectory.shape
    for step in range(num_steps):
        target_pos = trajectory[step]

        # Move the robot to the target position
        robot.position_control(target_pos)

        # Wait for the next timestep
        time.sleep(1 / freq)
