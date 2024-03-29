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

    use_sim_time = LaunchConfiguration('use_sim_time')

    mr_robot_xacro = os.path.join(pkg_object_follower_sim, "urdf", "mr_robot.xacro")
    assert os.path.exists(mr_robot_xacro), "No robot found in {}".format(mr_robot_xacro)

    mr_robot_desc_config = xacro.process_file(mr_robot_xacro)
    mr_robot_urdf = mr_robot_desc_config.toxml()

    mr_robot_sdf = os.path.join(pkg_object_follower_sim, "urdf", "mr_robot.sdf")
    urdf_path = os.path.join(pkg_object_follower_sim, "urdf", "mr_robot.urdf")

    spawn_robot = Node(package="ros_gz_sim", executable="create",
                    arguments=["-name", "mr_robot",
                                "-x", "0.0",
                                "-y", "3.0",
                                "-z", "0.0",
                                "-Y", "1.57",
                                "-file", mr_robot_sdf])

    # robot state publisher node
    state_publisher = Node(package='robot_state_publisher', executable='robot_state_publisher',
				output='screen',
				parameters = [
					{'ignore_timestamp': False},
                    {'use_sim_time': use_sim_time},
					{'use_tf_static': True},
					{'robot_description': open(urdf_path).read()}],
				arguments = [urdf_path])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value="true"),
        spawn_robot,
        state_publisher
    ])