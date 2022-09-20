from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    launch_d = LaunchDescription()
    
    pf400_client = Node(
            package = 'pf400_client',
            namespace = 'pf400_client',
            executable = 'pf400_client',
            output = "screen",
            name='pf400Node'
    )
    pf400_camera_client = Node(
            package = 'pf400_client',
            namespace = 'pf400_camera_client',
            executable = 'pf400_camera_client',
            output = "screen",
            name='pf400CameraNode'
    )

    launch_d.add_action(pf400_client)
    launch_d.add_action(pf400_camera_client)
    return launch_d
    