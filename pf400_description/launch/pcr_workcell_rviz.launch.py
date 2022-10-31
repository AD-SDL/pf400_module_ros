import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
 
def generate_launch_description():

  pkg_share = FindPackageShare(package='pf400_description').find('pf400_description')
  default_rviz_config_path = os.path.join(pkg_share, 'config/pcr_workcell_config.rviz') 
  default_urdf_model_path = os.path.join(pkg_share, 'urdf/pcr_workcell.urdf.xacro') 
 
  
  fake_hardware = LaunchConfiguration('fake_hardware')
  urdf_model = LaunchConfiguration('urdf_model')
  rviz_config_file = LaunchConfiguration('rviz_config_file')
  use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
  use_rviz = LaunchConfiguration('use_rviz')
  use_sim_time = LaunchConfiguration('use_sim_time')
 
  declare_urdf_model_path_cmd = DeclareLaunchArgument(
    name='urdf_model', 
    default_value=default_urdf_model_path, 
    description='Absolute path to robot urdf file')
     
  declare_rviz_config_file_cmd = DeclareLaunchArgument(
    name='rviz_config_file',
    default_value=default_rviz_config_path,
    description='Full path to the RVIZ config file to use')
     
  declare_use_fake_joint_state_pub_cmd = DeclareLaunchArgument(
    name='fake_hardware',
    default_value='False',
    description='Flag to enable joint_state_publisher_fake_hardware')
   
  declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
    name='use_robot_state_pub',
    default_value='True',
    description='Whether to start the robot state publisher')
 
  declare_use_rviz_cmd = DeclareLaunchArgument(
    name='use_rviz',
    default_value='True',
    description='Whether to start RVIZ')
     
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='False',
    description='Use simulation (Gazebo) clock if true')

 
  # A fake_hardware to manipulate the joint state values
  start_joint_state_publisher_fake_hardware_node = Node(
    condition=IfCondition(fake_hardware),
    package='joint_state_publisher_gui',
    executable='joint_state_publisher_gui',
    name='joint_state_publisher_gui')
 
  # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
  start_robot_state_publisher_cmd = Node(
    condition=IfCondition(use_robot_state_pub),
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'use_sim_time': use_sim_time, 'robot_description': Command(['xacro ', urdf_model])}],
    arguments=[default_urdf_model_path])
 
  # Launch RViz
  start_rviz_cmd = Node(
    condition=IfCondition(use_rviz),
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', rviz_config_file])

  # Start RealHarware Joint State Publisher Client
  start_pf400_description_client = Node(
    condition=UnlessCondition(fake_hardware),
    package = "pf400_description",
    executable = 'pf400_description_client',
    name = 'PF400DescriptionNode',
    output = 'screen'
  )
   
  # Create the launch description and populate
  ld = LaunchDescription()
 
  # Declare the launch options
  ld.add_action(declare_urdf_model_path_cmd)
  ld.add_action(declare_rviz_config_file_cmd)
  ld.add_action(declare_use_fake_joint_state_pub_cmd)
  ld.add_action(declare_use_robot_state_pub_cmd)  
  ld.add_action(declare_use_rviz_cmd) 
  ld.add_action(declare_use_sim_time_cmd)
 
  # Add any actions
  ld.add_action(start_joint_state_publisher_fake_hardware_node)
  ld.add_action(start_robot_state_publisher_cmd)
  ld.add_action(start_rviz_cmd)
  ld.add_action(start_pf400_description_client)
  
  return ld