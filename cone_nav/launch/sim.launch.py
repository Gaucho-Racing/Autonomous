from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("cone_nav")
    params_file = LaunchConfiguration("params_file")
    engine_path = LaunchConfiguration("engine_path")
    rviz_config = LaunchConfiguration("rviz_config")
    sim_type = LaunchConfiguration("sim_type")
    enable_orbslam = LaunchConfiguration("enable_orbslam")
    avoidance_enabled = LaunchConfiguration("avoidance_enabled")
    avoidance_shadow_mode = LaunchConfiguration("avoidance_shadow_mode")
    orbslam_vocabulary_file = LaunchConfiguration("orbslam_vocabulary_file")
    orbslam_settings_file = LaunchConfiguration("orbslam_settings_file")

    common_parameters = [
        params_file,
        {
            "use_sim": True,
            "trt_engine_path": engine_path,
            "avoidance_enabled": ParameterValue(avoidance_enabled, value_type=bool),
            "safety_enabled": ParameterValue(avoidance_enabled, value_type=bool),
            "avoidance_shadow_mode": ParameterValue(
                avoidance_shadow_mode, value_type=bool
            ),
        },
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution([pkg_share, "config", "params.yaml"]),
            ),
            DeclareLaunchArgument(
                "engine_path",
                default_value=PathJoinSubstitution([pkg_share, "models", "cone_yolo.engine"]),
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution([pkg_share, "config", "viz.rviz"]),
            ),
            DeclareLaunchArgument(
                "sim_type",
                default_value="f1tenth",
                description="Simulation bridge to launch: f1tenth or fsae",
            ),
            DeclareLaunchArgument("enable_orbslam", default_value="false"),
            DeclareLaunchArgument(
                "avoidance_enabled",
                default_value="false",
                description="Enable only when the legacy bridge publishes registered depth.",
            ),
            DeclareLaunchArgument("avoidance_shadow_mode", default_value="false"),
            DeclareLaunchArgument(
                "orbslam_vocabulary_file",
                default_value="/opt/orbslam3/Vocabulary/ORBvoc.txt",
            ),
            DeclareLaunchArgument(
                "orbslam_settings_file",
                default_value=PathJoinSubstitution(
                    [pkg_share, "config", "orbslam3_zed2i_stereo_inertial.yaml"]
                ),
            ),
            Node(
                package="f1tenth_gym_ros",
                executable="gym_bridge",
                name="f1tenth_gym_bridge",
                output="screen",
                condition=IfCondition(PythonExpression(["'", sim_type, "' == 'f1tenth'"])),
            ),
            Node(
                package="fsae_sim_bridge",
                executable="bridge_node",
                name="fsae_sim_bridge",
                output="screen",
                condition=IfCondition(PythonExpression(["'", sim_type, "' == 'fsae'"])),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_sim_camera_tf",
                arguments=[
                    "0.20",
                    "0.0",
                    "0.20",
                    "0.0",
                    "0.0",
                    "0.0",
                    "base_link",
                    "zed2i_left_camera_frame",
                ],
            ),
            Node(
                package="orbslam3",
                executable="stereo-inertial",
                name="orbslam3_stereo_inertial",
                output="screen",
                condition=IfCondition(enable_orbslam),
                arguments=[orbslam_vocabulary_file, orbslam_settings_file, "true"],
                remappings=[
                    ("/camera/left/image_raw", "/sim/camera/image_raw"),
                    ("/camera/right/image_raw", "/sim/camera/right/image_raw"),
                    ("/imu", "/sim/imu"),
                ],
            ),
            Node(
                package="cone_nav",
                executable="cone_detector_node",
                name="cone_detector_node",
                output="screen",
                parameters=common_parameters,
            ),
            Node(
                package="cone_nav",
                executable="cone_localizer_node",
                name="cone_localizer_node",
                output="screen",
                parameters=common_parameters,
            ),
            Node(
                package="cone_nav",
                executable="path_planner_node.py",
                name="path_planner_node",
                output="screen",
                parameters=common_parameters,
                remappings=[
                    ("/path", "/path/nominal"),
                    ("/path/markers", "/path/nominal_markers"),
                ],
            ),
            Node(
                package="cone_nav",
                executable="depth_obstacle_node",
                name="depth_obstacle_node",
                output="screen",
                parameters=common_parameters,
            ),
            Node(
                package="cone_nav",
                executable="obstacle_avoidance_node.py",
                name="obstacle_avoidance_node",
                output="screen",
                parameters=common_parameters,
            ),
            Node(
                package="cone_nav",
                executable="pure_pursuit_node",
                name="pure_pursuit_node",
                output="screen",
                parameters=common_parameters,
                remappings=[("/drive", "/drive_candidate")],
            ),
            Node(
                package="cone_nav",
                executable="drive_safety_node",
                name="drive_safety_node",
                output="screen",
                parameters=common_parameters,
            ),
            Node(
                package="cone_nav",
                executable="ackermann_to_twist_node",
                name="ackermann_to_twist_node",
                output="screen",
                condition=IfCondition(PythonExpression(["'", sim_type, "' == 'fsae'"])),
                parameters=common_parameters,
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
            ),
        ]
    )
