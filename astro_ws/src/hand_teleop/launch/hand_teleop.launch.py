from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    twist_mux_config = os.path.join(
        get_package_share_directory('astro'),
        'config',
        'twist_mux.yaml'
    )

    return LaunchDescription([

        Node(
            package='hand_teleop',
            executable='hand_capture_node',
            name='hand_capture_node',
            output='screen',
        ),

        Node(
            package='hand_teleop',
            executable='hand_teleop_node',
            name='hand_teleop_node',
            output='screen',
            parameters=[{
                'deadzone_radius'   : 0.03,
                'max_linear_speed'  : 1.0,
                'max_angular_speed' : 1.0,
                'map_range_linear'  : 0.20,
                'map_range_angular' : 0.20,
                'smoothing_alpha'   : 0.3,
                'tracking_timeout'  : 0.4,
                'publish_rate'      : 20.0,
            }]
        ),

        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[twist_mux_config],
            remappings=[('/cmd_vel_out', '/cmd_vel')],
        ),

        Node(
            package='hand_teleop_control',
            executable='obstacle_avoidance_node',
            name='obstacle_avoidance_node',
            output='screen',
        ),
    ])