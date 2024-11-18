import open3d as o3d
import numpy as np
from vrteleop.urdf_parser_v2 import URDFParserV2
from lerobot.common.robot2.robot import MotorData, Robot
from vrteleop.ik import IK


class VirtualRobot(Robot):
    def __init__(self, urdf_path: str, fk: IK):
        """
        Initialize the virtual robot for visualization.

        Args:
            urdf_path (str): Path to the URDF file.
            joint_count (int): Number of joints in the robot.
        """
        self.urdf_path = urdf_path
        self.parser = URDFParserV2(urdf_path)
        self.link_stl_map = self.parser.get_link_stl_map()
        self.joint_count = len(self.parser.get_links())
        self.ik = fk
        self.current_pos = np.zeros(self.joint_count)
        self.current_vel = np.zeros(self.joint_count)
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
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)  # Adjust size as needed
        self.visualizer.add_geometry(frame)

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
        self.current_pos = np.zeros(self.joint_count)
        self.current_vel = np.zeros(self.joint_count)

    def read(self) -> MotorData:
        """Read the current state of the robot."""
        return MotorData(pos=self.current_pos.copy(), vel=self.current_vel.copy())

    def position_control(self, target_pos: np.ndarray) -> None:
        """Move the robot to the specified target position."""
        if not self.connected:
            raise RuntimeError("VirtualRobot is not connected.")

        print(f"Moving to target position: {target_pos}")
        self.current_pos = target_pos

        # Compute forward kinematics
        fk_results = self.ik.fk(np.deg2rad(self.current_pos))

        # Update visualization
        for link_name, mesh in self.geometries.items():
            if link_name in fk_results:
                transform = fk_results[link_name]

                # Undo the current transformation
                current_transform = self.current_transformations[link_name]
                mesh.transform(np.linalg.inv(current_transform))

                # Apply the new transformation
                mesh.transform(transform)

                # Store the new transformation
                self.current_transformations[link_name] = transform

                # Update the geometry in the visualizer
                self.visualizer.update_geometry(mesh)
            else:
                print(f"Warning: No FK result for link {link_name}.")

        self.visualizer.poll_events()
        self.visualizer.update_renderer()
