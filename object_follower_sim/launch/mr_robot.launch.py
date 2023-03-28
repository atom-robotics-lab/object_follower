import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_object_follower_sim = get_package_share_directory("object_follower_sim")

    mr_robot_xacro = os.path.join(pkg_object_follower_sim, "urdf", "mr_robot.xacro")
    assert os.path.exists(mr_robot_xacro), "No robot found in {}".format(mr_robot_xacro)

    mr_robot_desc_config = xacro.process_file(mr_robot_xacro)
    mr_robot_urdf = mr_robot_desc_config.toxml()

    spawn_robot = Node(package="ros_gz_sim", executable="create",
                    arguments=["-name", "mr_robot",
                                "-x", "0.0",
                                "-y", "0.0",
                                "-z", "0.0",
                                "-Y", "1.57",
                                "-string", mr_robot_urdf])

    return LaunchDescription([
        spawn_robot
    ])