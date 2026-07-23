from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    config_file = PathJoinSubstitution(
        [FindPackageShare("robot_bringup"), "config", "robot.yaml"]
    )
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("robot_description"),
                    "launch",
                    "description.launch.py",
                ]
            )
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    common_parameters = [config_file, {"use_sim_time": use_sim_time}]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use the simulation clock",
            ),
            description_launch,
            Node(
                package="robot_demo",
                executable="robot_controller",
                name="robot_controller",
                output="screen",
                parameters=common_parameters,
            ),
            Node(
                package="robot_demo",
                executable="pose_monitor",
                name="pose_monitor",
                output="screen",
                parameters=common_parameters,
            ),
            Node(
                package="robot_demo",
                executable="robot_runtime_manager",
                name="robot_runtime_manager",
                output="screen",
                parameters=common_parameters,
            ),
        ]
    )
