import os
import time
from typing import Dict

import numpy as np

import open3d as o3d  # need to load open3d before pytorch
from vrteleop.ik import Kinematics
from vrteleop.ik_ws_server import ControllerData, IKServer
from vrteleop.vr_teleop_server import VRTeleopServer

from lerobot.common.robot2.virtual_robot import VirtualRobot
from lerobot.common.robot2.joint_traj import generate_joint_trajectory, play_joint_trajectory


HOST = '192.168.1.100'
WSS_PORT = 8765
APP_PORT = 5000
URDF_PATH = "/Users/parilo/devel/armlib/SO-ARM100/URDF/SO_5DOF_ARM100_8j_URDF.SLDASM/urdf/SO_5DOF_ARM100_8j_URDF.SLDASM.urdf"
END_LINK_NAME = "Fixed_Jaw"
CONTROL_FREQ = 25

SCRIPT_FOLDER = os.path.dirname(__file__)
SSL_CERT = os.path.join(SCRIPT_FOLDER, 'cert.pem')
SSL_KEY = os.path.join(SCRIPT_FOLDER, 'key.pem')


def main():

    np.set_printoptions(suppress=True, precision=4)

    ws_server = IKServer(
        host=HOST,
        port=WSS_PORT,
        cert_path=SSL_CERT,
        key_path=SSL_KEY,
    )
    ws_server.start()

    # Initialize and start the VRTeleop server
    server = VRTeleopServer(
        host=HOST,
        wss_port=WSS_PORT,
        app_port=APP_PORT,
        urdf_path=URDF_PATH,
        ssl_cert=SSL_CERT,
        ssl_key=SSL_KEY
    )
    server.start()

    kinematics = Kinematics(
        urdf_path=URDF_PATH,
        end_link_name=END_LINK_NAME,
        mod_matrix=np.eye(6)[:5],  # don't control rotation over z
    )

    robot = VirtualRobot(
        urdf_path=URDF_PATH,
        kinematics=kinematics,
        end_link_name=END_LINK_NAME,
        start_joints=np.array([0., 195, 182, 72.6855, 0]),
    )

    # Connect to the robot
    robot.connect()

    # Define relax position
    START_POS = np.array([0., 143, 129, 72.6855, 0])

    traj = generate_joint_trajectory(
        start_pos=robot.read().pos,
        end_pos=START_POS,
        max_joint_vel=90,
    )
    play_joint_trajectory(
        robot=robot,
        trajectory=traj,
    )

    prev_controller_data: ControllerData = None

    def teleop_callback(controller_data: ControllerData) -> Dict[str, np.ndarray]:
        nonlocal prev_controller_data

        print(controller_data.rightController.buttons)
        if prev_controller_data is not None:
            if (
                len(controller_data.rightController.buttons) > 4 and
                controller_data.rightController.buttons[5].pressed
            ):
                delta = np.array([
                    controller_data.rightController.position.x - prev_controller_data.rightController.position.x,
                    0, # controller_data.rightController.position.x - prev_controller_data.rightController.position.x,
                    0, # controller_data.rightController.position.y - prev_controller_data.rightController.position.y,
                    0,
                    0,
                    0,
                ])
                print(f'--- {delta}')

                cur_joints = robot.read().pos
                djoints = np.rad2deg(kinematics.djoints(
                    js=cur_joints,
                    dx=delta,
                ))
                # djoints = np.zeros_like(cur_joints)
                print(f'--- cur_joints {cur_joints}')
                robot.position_control(cur_joints + djoints)
        else:
            djoints = np.zeros_like(robot.read().pos)

        prev_controller_data = controller_data
        time.sleep(1. / CONTROL_FREQ)

        # print(f'--- joints 2 {robot.read().pos}')
        transforms = kinematics.fk(robot.read().pos)
        return transforms


    try:
        while True:
            ws_server.check(teleop_callback)
    finally:
        robot.relax()
        robot.disconnect()
        server.stop()
        ws_server.stop()


# Start the server
if __name__ == '__main__':
    main()


# Received data: {"leftController":{"position":{"x":-0.0969579741358757,"y":0.6429316997528076,"z":-0.27691584825515747},"rotation":{"x":29.686857880941417,"y":-13.466204055405289,"z":38.5308378329453},"buttons":[{"pressed":false,"value":0},{"pressed":false,"value":0},{"pressed":false,"value":0},{"pressed":false,"value":0},{"pressed":false,"value":0},{"pressed":false,"value":0},{"pressed":false,"value":0},{"pressed":false,"value":0}],"axes":[0,0,0,0]},"rightController":{"position":{"x":0.12382438033819199,"y":0.7657343745231628,"z":-0.22441256046295166},"rotation":{"x":51.77868081603406,"y":-1.4418945337878835,"z":-46.898254296107375},"buttons":[{"pressed":false,"value":0},{"pressed":false,"value":0},{"pressed":false,"value":0},{"pressed":false,"value":0},{"pressed":false,"value":0},{"pressed":false,"value":0},{"pressed":false,"value":0}],"axes":[0,0,0,0]}}
