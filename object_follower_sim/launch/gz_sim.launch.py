#!/usr/bin/env python

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import xacro

from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration




def generate_launch_description():

    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_object_follower_sim = get_package_share_directory("object_follower_sim")

    robot_arg = DeclareLaunchArgument('robot', default_value=TextSubstitution(text=''))

    namespace_arg    = DeclareLaunchArgument('namespace', default_value=TextSubstitution(text='r0'))
    
    X_launch_arg     = DeclareLaunchArgument('X',         default_value=TextSubstitution(text='0.0'))
    Y_launch_arg     = DeclareLaunchArgument('Y',         default_value=TextSubstitution(text='0.0'))
    Theta_launch_arg = DeclareLaunchArgument('Theta',     default_value=TextSubstitution(text='0.0'))

    models_dir = get_package_share_directory('object_follower_sim') + '/models'

    def create_robot_description(context): 
        xacro_file = os.path.join(get_package_share_directory('object_follower_sim'), 'urdf', context.launch_configurations['robot'], 'mr_robot.xacro')    
        assert os.path.exists(xacro_file), "The main.xacro doesnt exist in "+str(xacro_file)

        robot_description_config = xacro.process_file(xacro_file, 
            mappings={  "namespace": context.launch_configurations['namespace'], 
                        "models_dir": models_dir})
        
        robot_desc = robot_description_config.toxml()
        return [SetLaunchConfiguration('robot_desc', robot_desc)]

    create_robot_description_arg = OpaqueFunction(function=create_robot_description)

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


    spawn_robot = Node(package='object_follower_sim', 
                        name="publisher_robot", 
                        executable='gz_sim.launch.py', 
                        arguments=[LaunchConfiguration('robot_desc')],
                        parameters=[{
                            "X": '0',
                            "Y": '0',
                            "Theta": '0',
                            "namespace": 'r0'}],
                            output='screen')

    '''spawn_entity_node = Node(
        package='ignition_gazebo',
        executable='spawn_entity.py',
        arguments=['-entity', "MR ROBOT", '-x', '0', '-y', '0', '-z', '1', '-file', os.path.join(pkg_object_follower_sim, "urdf", "mr_robot.urdf")],
        output='screen',
    )'''

    spawn_robot_arg = Node(
    package='ros_ign_gazebo',
    executable='create',
    output='screen',
    arguments=["-file", os.path.join(pkg_object_follower_sim, "urdf", "mr_robot.xacro")]
    )

    return LaunchDescription([        
        #namespace_arg,
        #robot_arg,
        #X_launch_arg,
        #Y_launch_arg,
        #Theta_launch_arg,
        #create_robot_description_arg,
        gazebo,
        spawn_robot_arg,
    ])