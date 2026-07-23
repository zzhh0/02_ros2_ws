from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("robot_bringup"), "launch", "robot.launch.py"]
            )
        )
    )

    return LaunchDescription(
        [
            robot,
            Node(
                package="robot_demo",
                executable="cmd_publisher",
                name="cmd_publisher",
                output="screen",
            ),
        ]
    )
