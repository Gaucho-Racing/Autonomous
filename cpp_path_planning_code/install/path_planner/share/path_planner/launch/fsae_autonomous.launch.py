"""
FSAE Autonomous System Launch File

Launches the complete autonomous driving stack:
- Path Planner Node
- Pure Pursuit Controller Node

Usage:
    ros2 launch path_planner fsae_autonomous.launch.py
    
    # With custom parameters:
    ros2 launch path_planner fsae_autonomous.launch.py max_speed:=10.0 wheelbase:=0.30
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # =========================================================================
    # LAUNCH ARGUMENTS
    # =========================================================================
    
    # Vehicle parameters
    wheelbase_arg = DeclareLaunchArgument(
        'wheelbase',
        default_value='0.30',
        description='Vehicle wheelbase in meters (distance between axles)'
    )
    
    max_steering_arg = DeclareLaunchArgument(
        'max_steering_angle',
        default_value='0.40',
        description='Maximum steering angle in radians'
    )
    
    # Speed parameters
    max_speed_arg = DeclareLaunchArgument(
        'max_speed',
        default_value='8.0',
        description='Maximum vehicle speed in m/s'
    )
    
    min_speed_arg = DeclareLaunchArgument(
        'min_speed',
        default_value='2.0',
        description='Minimum vehicle speed in m/s'
    )
    
    # Planning parameters
    lookahead_arg = DeclareLaunchArgument(
        'lookahead_distance',
        default_value='15.0',
        description='Path planning lookahead distance in meters'
    )
    
    # Visualization
    publish_markers_arg = DeclareLaunchArgument(
        'publish_markers',
        default_value='true',
        description='Publish visualization markers for RViz'
    )
    
    # =========================================================================
    # NODES
    # =========================================================================
    
    # Path Planner Node
    path_planner_node = Node(
        package='path_planner',
        executable='planner_node',
        name='path_planner',
        output='screen',
        parameters=[{
            'max_speed': LaunchConfiguration('max_speed'),
            'min_speed': LaunchConfiguration('min_speed'),
            'lookahead_distance': LaunchConfiguration('lookahead_distance'),
            'max_edge_length': 8.0,
            'smoothing_resolution': 0.2,
            'max_lateral_accel': 6.0,
            'max_longitudinal_accel': 4.0,
            'max_longitudinal_decel': 6.0,
            'expected_track_width': 3.0,
            'publish_markers': LaunchConfiguration('publish_markers'),
            'planning_rate_hz': 20.0,
        }],
        remappings=[
            # Add any topic remappings here
            # ('/detected_cones', '/perception/cones'),
        ]
    )
    
    # Pure Pursuit Controller Node
    controller_node = Node(
        package='pure_pursuit',
        executable='controller_node',
        name='pure_pursuit_controller',
        output='screen',
        parameters=[{
            'wheelbase': LaunchConfiguration('wheelbase'),
            'max_steering_angle': LaunchConfiguration('max_steering_angle'),
            'min_lookahead': 1.5,
            'max_lookahead': 6.0,
            'lookahead_gain': 0.6,
            'speed_kp': 2.0,
            'speed_ki': 0.5,
            'speed_kd': 0.1,
            'use_curvature_feedforward': True,
            'curvature_ff_gain': 0.8,
            'max_speed': LaunchConfiguration('max_speed'),
            'emergency_stop_distance': 0.5,
            'control_rate_hz': 50.0,
            'publish_markers': LaunchConfiguration('publish_markers'),
        }],
        remappings=[
            # Add any topic remappings here
            # ('/odom', '/localization/odom'),
            # ('/drive', '/vesc/ackermann_cmd'),
        ]
    )
    
    # =========================================================================
    # LAUNCH DESCRIPTION
    # =========================================================================
    
    return LaunchDescription([
        # Arguments
        wheelbase_arg,
        max_steering_arg,
        max_speed_arg,
        min_speed_arg,
        lookahead_arg,
        publish_markers_arg,
        
        # Nodes
        path_planner_node,
        controller_node,
    ])
