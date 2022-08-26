from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pf400_module_client',
            namespace='pf400_module',
            executable='pf400ControlNode',
            name='pf400Node'
        ),
    ])