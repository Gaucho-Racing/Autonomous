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
    drive_enabled = LaunchConfiguration("drive_enabled")
    avoidance_enabled = LaunchConfiguration("avoidance_enabled")
    avoidance_shadow_mode = LaunchConfiguration("avoidance_shadow_mode")
    depth_mode = LaunchConfiguration("depth_mode")
    enable_rviz = LaunchConfiguration("enable_rviz")
    enable_orbslam = LaunchConfiguration("enable_orbslam")
    orbslam_vocabulary_file = LaunchConfiguration("orbslam_vocabulary_file")
    orbslam_settings_file = LaunchConfiguration("orbslam_settings_file")

    common_parameters = [
        params_file,
        {
            "use_sim": True,
            "use_sim_time": True,
            "trt_engine_path": engine_path,
            "drive_enabled": ParameterValue(drive_enabled, value_type=bool),
            "avoidance_enabled": ParameterValue(avoidance_enabled, value_type=bool),
            "safety_enabled": ParameterValue(avoidance_enabled, value_type=bool),
            "avoidance_shadow_mode": ParameterValue(
                avoidance_shadow_mode, value_type=bool
            ),
            "sim_depth_topic": ParameterValue(
                PythonExpression(
                    [
                        "'/sim/camera/stereo_depth' if '",
                        depth_mode,
                        "' == 'stereo' else '/sim/camera/depth'",
                    ]
                ),
                value_type=str,
            ),
        },
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [pkg_share, "config", "isaac_params.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "engine_path",
                default_value=PathJoinSubstitution(
                    [pkg_share, "models", "cone_yolo.engine"]
                ),
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [pkg_share, "config", "viz_isaac.rviz"]
                ),
            ),
            DeclareLaunchArgument(
                "drive_enabled",
                default_value="false",
                description="Arm Ackermann output. Keep false until sensor/TF checks pass.",
            ),
            DeclareLaunchArgument("enable_rviz", default_value="true"),
            DeclareLaunchArgument("avoidance_enabled", default_value="true"),
            DeclareLaunchArgument("avoidance_shadow_mode", default_value="false"),
            DeclareLaunchArgument(
                "depth_mode",
                default_value="ground_truth",
                description="Depth source: ground_truth or stereo",
            ),
            DeclareLaunchArgument("enable_orbslam", default_value="false"),
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
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_isaac_left_optical_tf",
                arguments=[
                    "--x", "0.20",
                    "--y", "0.06",
                    "--z", "0.20",
                    "--qx", "0.5",
                    "--qy", "-0.5",
                    "--qz", "0.5",
                    "--qw", "-0.5",
                    "--frame-id", "base_link",
                    "--child-frame-id", "zed2i_left_camera_optical_frame",
                ],
                parameters=[{"use_sim_time": True}],
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
                parameters=[{"use_sim_time": True}],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="isaac_left_to_right_optical_tf",
                arguments=[
                    "--x", "0.12",
                    "--y", "0.0",
                    "--z", "0.0",
                    "--qx", "0.0",
                    "--qy", "0.0",
                    "--qz", "0.0",
                    "--qw", "1.0",
                    "--frame-id", "zed2i_left_camera_optical_frame",
                    "--child-frame-id", "zed2i_right_camera_optical_frame",
                ],
                parameters=[{"use_sim_time": True}],
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
                executable="stereo_depth_node",
                name="stereo_depth_node",
                output="screen",
                condition=IfCondition(
                    PythonExpression(["'", depth_mode, "' == 'stereo'"])
                ),
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
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                condition=IfCondition(enable_rviz),
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": True}],
            ),
        ]
    )
