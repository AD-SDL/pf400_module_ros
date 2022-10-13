from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    pkg_name = 'pf400_description'
    pkg_dir = os.popen('/bin/bash -c "source /usr/share/colcon_cd/function/colcon_cd.sh && colcon_cd %s && pwd"' % pkg_name).read().strip()

    urdf_file_name = 'urdf/PF400.urdf'

    urdf = os.path.join(get_package_share_directory('pf400_description'),urdf_file_name)
    rviz_config =  os.path.join(pkg_dir, 'config', 'StandardSettings.rviz')   
    urdf_model = LaunchConfiguration('urdf_model')

    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='PF400rviz2',
            parameters=['-d', [rviz_config]],
            arguments = [urdf]
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher'
        ),
        Node(
            package='pf400_description',
            executable='joint_publisher',
            name='pf400_joint_publisher',
            output='screen'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'use_sim_time': use_sim_time, 
            'robot_description': Command(['xacro ', urdf_model])}],
            arguments=[urdf])

    ])

    