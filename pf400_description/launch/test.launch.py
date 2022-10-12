from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    pkg_name = 'pf400_description'
    pkg_dir = os.popen('/bin/bash -c "source /usr/share/colcon_cd/function/colcon_cd.sh && colcon_cd %s && pwd"' % pkg_name).read().strip()

    urdf_file_name = 'urdf/PF400.urdf'

    urdf = os.path.join(get_package_share_directory('pf400_description'),urdf_file_name)
    rviz_config =  os.path.join(pkg_dir, 'config', 'StandardSettings.rviz')   

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
        )
    ])

    