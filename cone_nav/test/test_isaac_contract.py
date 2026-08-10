import ast
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parent


class IsaacIntegrationContractTest(unittest.TestCase):
    def test_python_sources_parse(self):
        sources = (
            PACKAGE_ROOT / "launch" / "isaac.launch.py",
            PACKAGE_ROOT / "cone_nav" / "path_planner_node.py",
            PACKAGE_ROOT / "cone_nav" / "obstacle_avoidance_node.py",
            PACKAGE_ROOT / "cone_nav" / "avoidance_algorithms.py",
            REPOSITORY_ROOT / "isaac_sim" / "autonomousgr_scene.py",
        )
        for source in sources:
            ast.parse(source.read_text(encoding="utf-8"), filename=str(source))

    def test_isaac_topics_match_navigation_parameters(self):
        params = (PACKAGE_ROOT / "config" / "isaac_params.yaml").read_text(
            encoding="utf-8"
        )
        scene = (REPOSITORY_ROOT / "isaac_sim" / "autonomousgr_scene.py").read_text(
            encoding="utf-8"
        )
        for topic in (
            "/sim/camera/image_raw",
            "/sim/camera/right/image_raw",
            "/sim/camera/depth",
            "/sim/camera/camera_info",
        ):
            self.assertIn(topic, params)
            self.assertIn(topic, scene)

    def test_isaac_launch_is_safe_and_uses_sim_time(self):
        launch = (PACKAGE_ROOT / "launch" / "isaac.launch.py").read_text(
            encoding="utf-8"
        )
        self.assertIn('default_value="false"', launch)
        self.assertIn('"use_sim_time": True', launch)
        self.assertNotIn('"/cmd_vel"', launch)
        scene = (REPOSITORY_ROOT / "isaac_sim" / "autonomousgr_scene.py").read_text(
            encoding="utf-8"
        )
        self.assertIn('"/drive"', scene)
        self.assertIn('"/drive_candidate"', launch)
        self.assertIn('executable="drive_safety_node"', launch)

    def test_optical_frame_exists_and_is_selected(self):
        urdf = PACKAGE_ROOT / "urdf" / "f1tenth_base.urdf.xacro"
        ET.parse(urdf)
        text = urdf.read_text(encoding="utf-8")
        params = (PACKAGE_ROOT / "config" / "isaac_params.yaml").read_text(
            encoding="utf-8"
        )
        self.assertIn("zed2i_left_camera_optical_frame", text)
        self.assertIn("zed2i_right_camera_optical_frame", text)
        self.assertIn('xyz="0.12 0 0"', text)
        self.assertIn('camera_frame: "zed2i_left_camera_optical_frame"', params)

    def test_stale_path_and_controller_timeouts_are_configured(self):
        params = (PACKAGE_ROOT / "config" / "isaac_params.yaml").read_text(
            encoding="utf-8"
        )
        planner = (PACKAGE_ROOT / "cone_nav" / "path_planner_node.py").read_text(
            encoding="utf-8"
        )
        controller = (PACKAGE_ROOT / "src" / "pure_pursuit_node.cpp").read_text(
            encoding="utf-8"
        )
        self.assertIn("path_hold_timeout_sec", params)
        self.assertIn("path_hold_timeout_sec", planner)
        self.assertIn("path_timeout_sec", controller)
        self.assertIn("drive_enabled", controller)

    def test_obstacle_topics_and_nodes_are_wired(self):
        params = (PACKAGE_ROOT / "config" / "isaac_params.yaml").read_text(
            encoding="utf-8"
        )
        launch = (PACKAGE_ROOT / "launch" / "isaac.launch.py").read_text(
            encoding="utf-8"
        )
        cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        for topic in ("/obstacles/local_grid", "/obstacles/points"):
            self.assertIn(topic, params)
        for executable in (
            "stereo_depth_node",
            "depth_obstacle_node",
            "obstacle_avoidance_node.py",
            "drive_safety_node",
        ):
            self.assertIn(executable, launch)
            self.assertIn(executable, cmake)
        self.assertIn('("/path", "/path/nominal")', launch)
        self.assertIn('("/drive", "/drive_candidate")', launch)
        self.assertIn('default_value="ground_truth"', launch)
        self.assertIn("/sim/camera/stereo_depth", launch)

    def test_isaac_scene_has_collision_scenarios(self):
        scene = (REPOSITORY_ROOT / "isaac_sim" / "autonomousgr_scene.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("UsdPhysics.CollisionAPI.Apply", scene)
        self.assertIn("--obstacle-scenario", scene)
        for scenario in ("clear", "center", "right", "narrow", "blocked"):
            self.assertIn(f'"{scenario}"', scene)


if __name__ == "__main__":
    unittest.main()
