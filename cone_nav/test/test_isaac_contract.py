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

    def test_optical_frame_exists_and_is_selected(self):
        urdf = PACKAGE_ROOT / "urdf" / "f1tenth_base.urdf.xacro"
        ET.parse(urdf)
        text = urdf.read_text(encoding="utf-8")
        params = (PACKAGE_ROOT / "config" / "isaac_params.yaml").read_text(
            encoding="utf-8"
        )
        self.assertIn("zed2i_left_camera_optical_frame", text)
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


if __name__ == "__main__":
    unittest.main()
