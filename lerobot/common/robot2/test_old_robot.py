import time
import torch as t
import numpy as np
from lerobot.common.robot_devices.robots.factory import make_robot
from lerobot.common.utils.utils import init_hydra_config

robot_path = 'lerobot/configs/robot/so100.yaml'
robot_overrides = []

robot_cfg = init_hydra_config(robot_path, robot_overrides)
robot = make_robot(robot_cfg)

robot.connect()

RELAX_POS = t.as_tensor(np.array([ -0.3516, 195.6445, 182.3730,  72.6855,  -0.3516,   0.8257]))
robot.send_action(RELAX_POS)
time.sleep(1)

observation = robot.capture_observation()
print(f'--- obs {observation}')
target_pos = observation['observation.state']

np.set_printoptions(suppress=True, precision=4)
t.set_printoptions(sci_mode=False, precision=4)

num_steps = 300
freq = 25
for ind in range(num_steps):
    target_pos[5] = 45 - 45 * t.cos(t.tensor(2 * ind / freq))
    target_pos[3] = 36 + 36 * t.cos(t.tensor(2 * ind / freq))
    target_pos[2] = 151 + 31 * t.cos(t.tensor(2 * ind / freq))
    target_pos[1] = 170 + 25 * t.cos(t.tensor(2 * ind / freq))
    print(target_pos)
    robot.send_action(target_pos)
    time.sleep(1 / freq)

time.sleep(1)

robot.send_action(RELAX_POS)
time.sleep(1)

robot.disconnect()
