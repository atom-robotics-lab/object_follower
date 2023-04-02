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
    pkg_object_follower = get_package_share_directory("object_follower")

    config = os.path.join(pkg_object_follower,'config','params.yaml')

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
    
    parameter_bridge = Node(package="ros_gz_bridge", executable="parameter_bridge",
                            parameters = [
                                {'config_file': os.path.join(pkg_object_follower_sim, "config", "bridge.yaml")}
                                ],
                    )
    
    object_follower_node=Node(
        package = 'object_follower',
        name = 'object_follower',
        executable = 'object_follower',
        parameters = [config]
    )

    return LaunchDescription([
        gazebo,
        robot,
        parameter_bridge,
        object_follower_node
    ])