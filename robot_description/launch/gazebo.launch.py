from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    model_name = LaunchConfiguration("model_name")
    initial_z = LaunchConfiguration("z")

    xacro_file = PathJoinSubstitution([FindPackageShare("robot_description"),"rviz","robot.urdf.xacro"])

    robot_description = ParameterValue(
        Command(
            [FindExecutable(name="xacro")," ",xacro_file]
        ),
        value_type=str
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"),"launch","gz_sim.launch.py"]
            )
        ),
        launch_arguments={"gz_args":"-r empty.sdf"}.items()
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic","robot_description",
                   "-name",model_name,
                   "-z",initial_z,
                   "-allow_renaming","true"
                   ]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description":robot_description}
        ]
    )
    return LaunchDescription([
        DeclareLaunchArgument("model_name",default_value="robot_base",description="Gazebo entity name"),
        DeclareLaunchArgument("z",default_value="0.1",description="Gazebo entity name"),
        gazebo,
        robot_state_publisher,
        spawn_robot]
    )