import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():

    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_object_follower_sim = get_package_share_directory("object_follower_sim")

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
                ),
                launch_arguments={
                    'gz_args' : PathJoinSubstitution([
                        pkg_object_follower_sim,
                        'worlds',
                        'four_wall.sdf'
                    ])
                }.items(),  
            )                 


    spawn_entity_node = Node(
        package='ignition_gazebo',
        executable='spawn_entity.py',
        arguments=['-entity', "MR ROBOT", '-x', '0', '-y', '0', '-z', '1', '-file', os.path.join(pkg_object_follower_sim, "urdf", "mr_robot.urdf")],
        output='screen',
    )

    world = DeclareLaunchArgument(
            'world',
            default_value=[os.path.join(pkg_object_follower_sim, 'worlds', 'four_wall.sdf'), ''],
            description='SDF world file',
        )
    return LaunchDescription([
        gazebo,
        #spawn_entity_node,
        world
    ])