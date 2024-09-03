from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "bridge_mros2ros",
            default_value="true",
            description="Boolean flag to indicate whether to bridge mros to ros.",
        ),
        DeclareLaunchArgument(
            "bridge_ros2mros",
            default_value="false",
            description="Boolean flag to indicate whether to bridge ros to mros.",
        ),
        DeclareLaunchArgument(
            "ros2mros_execlude",
            default_value="",
            description="Execlude topics, topics are separated by ';'",
        ),
        DeclareLaunchArgument(
            "ros2mros_include",
            default_value="",
            description="Include topics, topics are separated by ';'",
        ),
        DeclareLaunchArgument(
            "mros2ros_execlude",
            default_value="",
            description="Execlude topics, topics are separated by ';'",
        ),
        DeclareLaunchArgument(
            "mros2ros_include",
            default_value="",
            description="Include topics, topics are separated by ';'",
        ),
        Node(
            package='mrosbridger',
            executable='mrosbridger',
            output='screen',
            parameters=[
                {
                    "bridge_mros2ros": LaunchConfiguration("bridge_mros2ros"),
                    "bridge_ros2mros": LaunchConfiguration("bridge_ros2mros"),
                    "ros2mros_execlude": LaunchConfiguration("ros2mros_execlude"),
                    "ros2mros_include": LaunchConfiguration("ros2mros_include"),
                    "mros2ros_execlude": LaunchConfiguration("mros2ros_execlude"),
                    "mros2ros_include": LaunchConfiguration("mros2ros_include"),
                },
            ],
        ),
    ])