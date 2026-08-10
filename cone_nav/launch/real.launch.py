from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("cone_nav")
    params_file = LaunchConfiguration("params_file")
    engine_path = LaunchConfiguration("engine_path")
    rviz_config = LaunchConfiguration("rviz_config")
    enable_orbslam = LaunchConfiguration("enable_orbslam")
    avoidance_enabled = LaunchConfiguration("avoidance_enabled")
    avoidance_shadow_mode = LaunchConfiguration("avoidance_shadow_mode")
    left_image_topic = LaunchConfiguration("left_image_topic")
    right_image_topic = LaunchConfiguration("right_image_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    orbslam_vocabulary_file = LaunchConfiguration("orbslam_vocabulary_file")
    orbslam_settings_file = LaunchConfiguration("orbslam_settings_file")

    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("zed_wrapper"), "launch", "zed_camera.launch.py"])
        ),
        launch_arguments={
            "camera_model": "zed2i",
        }.items(),
    )

    common_parameters = [
        params_file,
        {
            "use_sim": False,
            "trt_engine_path": engine_path,
            "avoidance_enabled": ParameterValue(avoidance_enabled, value_type=bool),
            "safety_enabled": ParameterValue(avoidance_enabled, value_type=bool),
            "avoidance_shadow_mode": ParameterValue(
                avoidance_shadow_mode, value_type=bool
            ),
            "real_image_topic": left_image_topic,
            "real_right_image_topic": right_image_topic,
            "real_depth_topic": depth_topic,
            "real_camera_info_topic": camera_info_topic,
            "real_imu_topic": imu_topic,
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
            DeclareLaunchArgument("enable_orbslam", default_value="true"),
            DeclareLaunchArgument("avoidance_enabled", default_value="true"),
            DeclareLaunchArgument("avoidance_shadow_mode", default_value="false"),
            DeclareLaunchArgument(
                "left_image_topic",
                default_value="/zed2i/zed_node/left/image_rect_color",
            ),
            DeclareLaunchArgument(
                "right_image_topic",
                default_value="/zed2i/zed_node/right/image_rect_color",
            ),
            DeclareLaunchArgument(
                "depth_topic",
                default_value="/zed2i/zed_node/depth/depth_registered",
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/zed2i/zed_node/left/camera_info",
            ),
            DeclareLaunchArgument(
                "imu_topic",
                default_value="/zed2i/zed_node/imu/data",
            ),
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
            zed_launch,
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_zed_left_tf",
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
                    ("/camera/left/image_raw", left_image_topic),
                    ("/camera/right/image_raw", right_image_topic),
                    ("/imu", imu_topic),
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
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
            ),
        ]
    )
