from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    model_name = LaunchConfiguration("model_name")
    initial_z = LaunchConfiguration("z")
    world = LaunchConfiguration("world")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={"gz_args": ["-r ", world]}.items(),
    )
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("robot_bringup"), "launch", "robot.launch.py"]
            )
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            model_name,
            "-z",
            initial_z,
            "-allow_renaming",
            "true",
        ],
    )
    scan_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model_name",
                default_value="robot_base",
                description="Gazebo entity name",
            ),
            DeclareLaunchArgument(
                "z",
                default_value="0.1",
                description="Initial robot height in metres",
            ),
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("robot_bringup"), "worlds", "empty.sdf"]
                ),
                description="Gazebo world file",
            ),
            gazebo,
            robot,
            spawn_robot,
            scan_bridge,
        ]
    )
