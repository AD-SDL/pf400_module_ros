from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    launch_d = LaunchDescription()
    
    pf400_client = Node(
            package = 'pf400_client',
            namespace = 'std_ns',
            executable = 'pf400_client',
            output = "screen",
            name='pf400Node'
    )

    launch_d.add_action(pf400_client)
    return launch_d
    