from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='robot_demo',
                executable='cmd_publisher',
                name="cmd_publisher",
                output = 'screen'
            ),
            Node(
                package='robot_demo',
                executable='robot_controller',
                name='robot_controller',
                output='screen',
                parameters=[
                    {
                        'max_linear_speed':1.0,
                        'max_angular_speed':1.0
                    }
                ]
            ),
            Node(
                package='robot_demo',
                executable='pose_monitor',
                name='pose_monitor',
                output='screen'
            )
        ]
    )