from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("cone_nav")
    params_file = LaunchConfiguration("params_file")
    engine_path = LaunchConfiguration("engine_path")
    rviz_config = LaunchConfiguration("rviz_config")
    sim_type = LaunchConfiguration("sim_type")

    common_parameters = [
        params_file,
        {
            "use_sim": True,
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
            DeclareLaunchArgument(
                "sim_type",
                default_value="f1tenth",
                description="Simulation bridge to launch: f1tenth or fsae",
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
                remappings=[
                    (
                        "/drive",
                        PythonExpression(
                            ["'/drive' if '", sim_type, "' == 'f1tenth' else '/cmd_vel'"]
                        ),
                    )
                ],
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
