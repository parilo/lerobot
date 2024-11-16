# from lerobot.common.robot_devices.motors.dynamixel import DynamixelMotorsBus

# follower_port = "/dev/tty.usbmodem203NTEP8M1882"

# follower_arm = DynamixelMotorsBus(
#     port=follower_port,
#     motors={
#         # name: (index, model)
#         "shoulder_pan": (1, "sts3215"),
#         "shoulder_lift": (2, "sts3215"),
#         "elbow_flex": (3, "sts3215"),
#         "wrist_flex": (4, "sts3215"),
#         "wrist_roll": (5, "sts3215"),
#         "gripper": (6, "sts3215"),
#     },
# )

import time
import numpy as np
from lerobot.common.robot_devices.motors.feetech import FeetechMotorsBus


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


if not servos.is_connected:
    servos.connect()
    print('--- connected')

present_pos = servos.read("Present_Position")
print(f'--- present_pos {present_pos}')

goal_pos = np.array([1000, 1000, 1000, 1000, 1000, 1000]).astype(np.int32)
servos.write("Goal_Position", goal_pos)
print(f'--- done')
time.sleep(1)

present_pos = servos.read("Present_Position")
print(f'--- present_pos {present_pos}')

goal_pos = np.array([2000, 2000, 2000, 2000, 2000, 2000]).astype(np.int32)
servos.write("Goal_Position", goal_pos)
print(f'--- done')
time.sleep(1)

present_pos = servos.read("Present_Position")
print(f'--- present_pos {present_pos}')

if servos.is_connected:
    # Disconnect manually to avoid a "Core dump" during process
    # termination due to camera threads not properly exiting.
    servos.disconnect()
    print('--- disconnected')

# python lerobot/scripts/configure_motor.py --port /dev/tty.usbmodem58CD1772711 --brand feetech --model sts3215 --baudrate 1000000 --ID 1
