from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
import os
from launch import LaunchDescription

from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
# import pf400_description 


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_file_name = 'urdf/PF400urdf'

    urdf = os.path.join(
        get_package_share_directory('pf400_description'),
        urdf_file_name)

    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        # Node(
        #     package='pf400_description',
        #     namespace='pf400',
        #     executable='pf400_joint_pub',
        #     name='pf400_joint_publisher'
        # ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='pf400DescriptionNode',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,'robot_description': robot_desc}],
            arguments=[urdf]
        ),
        # Node(
        #     package='rviz2',
        #     namespace='pf400',
        #     executable='rviz2',
        #     name='pf400_viz',
        #     arguments= ['-d', [os.path.join('pf400_description', 'config', 'StandardSettings.rviz')]]
        # ),
    ])