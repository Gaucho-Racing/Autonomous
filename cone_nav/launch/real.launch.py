from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("cone_nav")
    params_file = LaunchConfiguration("params_file")
    engine_path = LaunchConfiguration("engine_path")
    rviz_config = LaunchConfiguration("rviz_config")
    enable_orbslam = LaunchConfiguration("enable_orbslam")
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
                    ("/camera/left/image_raw", "/zed2i/zed_node/left/image_rect_color"),
                    ("/camera/right/image_raw", "/zed2i/zed_node/right/image_rect_color"),
                    ("/imu", "/zed2i/zed_node/imu/data"),
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
            ),
            Node(
                package="cone_nav",
                executable="pure_pursuit_node",
                name="pure_pursuit_node",
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
