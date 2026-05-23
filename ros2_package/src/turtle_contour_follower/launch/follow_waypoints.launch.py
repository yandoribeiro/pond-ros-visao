from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtle_contour_follower',
            executable='follow_waypoints_node',
            name='follow_waypoints_node',
            parameters=[{
                'waypoints_file': '',
                'linear_speed': 4.0,
                'angular_speed': 12.0,
                'goal_tolerance': 0.20,
                'angular_gain': 8.0,
                'heading_gate': 1.0,
                'min_point_spacing': 0.04,
            }],
        )
    ])
