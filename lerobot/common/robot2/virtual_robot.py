import open3d as o3d
import numpy as np
from typing import Optional
from vrteleop.urdf_parser_v2 import URDFParserV2
from lerobot.common.robot2.robot import MotorData, Robot
from vrteleop.ik import Kinematics


class VirtualRobot(Robot):
    def __init__(
            self, urdf_path: str,
            kinematics: Kinematics,
            end_link_name: str,
            start_joints: Optional[np.ndarray] = None,
            ):
        """
        Initialize the virtual robot for visualization.

        Args:
            urdf_path (str): Path to the URDF file.
            joint_count (int): Number of joints in the robot.
        """
        self.urdf_path = urdf_path
        self.parser = URDFParserV2(urdf_path)
        self.link_stl_map = self.parser.get_link_stl_map()
        self.kinematics = kinematics
        self.joint_count = kinematics.num_dof
        self.end_link_name = end_link_name
        self.end_link_frame = None
        self.end_link_frame_tr = np.eye(4)
        self.current_jpos = (
            np.zeros(self.joint_count)
            if start_joints is None
            else start_joints
        )
        self.current_jvel = np.zeros(self.joint_count)
        self.visualizer = o3d.visualization.Visualizer()
        self.geometries = {}
        self.current_transformations = {}
        self.connected = False

    def connect(self) -> None:
        """Initialize the Open3D visualizer and load geometries."""
        if self.connected:
            print("Robot is already connected.")
            return

        self.visualizer.create_window()

        for link_name, stl_path in self.link_stl_map.items():
            try:
                mesh = o3d.io.read_triangle_mesh(stl_path)
                if mesh.is_empty():
                    print(f"Warning: STL file {stl_path} is empty.")
                    continue
                mesh.compute_vertex_normals()
                self.visualizer.add_geometry(mesh)
                self.geometries[link_name] = mesh
                self.current_transformations[link_name] = np.eye(4)

            except Exception as e:
                print(f"Error loading STL {stl_path}: {e}")

        # Create a coordinate frame for the link
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
        self.visualizer.add_geometry(frame)

        # Create a coordinate frame for the end link
        self.end_link_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
        self.visualizer.add_geometry(self.end_link_frame)

        self.connected = True

    def disconnect(self) -> None:
        """Close the Open3D visualizer."""
        if not self.connected:
            print("Robot is already disconnected.")
            return

        self.visualizer.destroy_window()
        self.geometries.clear()
        self.current_transformations.clear()
        self.connected = False

    def relax(self) -> None:
        """Set the robot to a relaxed state."""
        self.current_jpos = np.zeros(self.joint_count)
        self.current_jvel = np.zeros(self.joint_count)

    def read(self) -> MotorData:
        """Read the current state of the robot."""
        return MotorData(pos=self.current_jpos.copy(), vel=self.current_jvel.copy())

    def position_control(self, target_pos: np.ndarray) -> None:
        """Move the robot to the specified target position."""
        if not self.connected:
            raise RuntimeError("VirtualRobot is not connected.")

        print(f"Moving to target position: {target_pos}")
        self.current_jpos = target_pos

        # Compute forward kinematics
        fk_results = self.kinematics.fk(self.current_jpos)

        # Update visualization
        for link_name, mesh in self.geometries.items():
            if link_name in fk_results:
                transform = fk_results[link_name]

                # Undo the current transformation
                current_transform = self.current_transformations[link_name]

                # Apply the new transformation
                mesh.transform(transform @ np.linalg.inv(current_transform))

                # Store the new transformation
                self.current_transformations[link_name] = transform
                if link_name == self.end_link_name:
                    self.end_link_frame.transform(transform @ np.linalg.inv(self.end_link_frame_tr))
                    self.end_link_frame_tr = transform
                    self.visualizer.update_geometry(self.end_link_frame)

                # Update the geometry in the visualizer
                self.visualizer.update_geometry(mesh)
            else:
                print(f"Warning: No FK result for link {link_name}.")

        self.visualizer.poll_events()
        self.visualizer.update_renderer()
