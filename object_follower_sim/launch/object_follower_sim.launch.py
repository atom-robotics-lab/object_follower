import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_object_follower_sim = get_package_share_directory("object_follower_sim")

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
                ),
                launch_arguments={
                    'gz_args' : os.path.join(
                        pkg_object_follower_sim,
                        'worlds',
                        'four_wall.sdf'
                    )
                }.items()          
            )
    
    robot = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_object_follower_sim, "launch", "mr_robot.launch.py")
                )   
            )

    return LaunchDescription([
        gazebo,
        robot
    ])