from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Robot ip address
    os.environ['MROS_AGENT_IP'] = '10.192.1.2'

    return LaunchDescription([
        DeclareLaunchArgument(
            "bridge_mros2ros",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "mros2ros_include",
            default_value="/ImuData;/RobotCmdPointFoot;/RobotStatePointFoot",
        ),
        Node(
            package='mrosbridger',
            executable='mrosbridger',
            output='screen',
            parameters=[
                {
                    "bridge_mros2ros": LaunchConfiguration("bridge_mros2ros"),
                    "mros2ros_include": LaunchConfiguration("mros2ros_include"),
                },
            ],
        ),
        Node(
            package='plotjuggler',
            executable='plotjuggler',
        ),
    ])