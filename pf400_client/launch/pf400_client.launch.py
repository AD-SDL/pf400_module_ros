
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ip = LaunchConfiguration('ip')
    port = LaunchConfiguration('port')

    declare_use_ip_cmd = DeclareLaunchArgument(
        name='ip',
        default_value="146.137.240.35",
        description='Flag to accept ip address')

    declare_use_port_cmd = DeclareLaunchArgument(
        name='port',
        default_value="10100",
        description='Flag to accept port number')

    pf400_client = Node(
            package = 'pf400_client',
            namespace = 'std_ns',
            executable = 'pf400_client',
            output = "screen",
            name='pf400Node',
            parameters=[
                {'ip':ip},
                {'port':port}
                ],
            emulate_tty=True

    )

    launch_d = LaunchDescription()

    launch_d.add_action(declare_use_ip_cmd)
    launch_d.add_action(declare_use_port_cmd)
    launch_d.add_action(pf400_client)
    
    return launch_d
    
