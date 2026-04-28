from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtle_tf_demo',
            executable='follow_waypoints_param',
            name='follow_waypoints_param',
            output='screen',
            parameters=[{
                'waypoints_x': [1.0, 2.0, 2.0, 1.0],
                'waypoints_y': [1.0, 1.0, 2.0, 2.0],
                'linear_speed': 0.12,
                'angular_speed': 0.25,
                'goal_tolerance': 0.15,
                'yaw_tolerance': 0.20,
            }]
        )
    ])
