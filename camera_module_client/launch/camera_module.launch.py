from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    launch_d = LaunchDescription()

    plate_camera_client = Node(
            package = 'plate_camera_client',
            namespace = 'plate_camera_client',
            executable = 'plate_camera_client',
            output = "screen",
            name='plateCameraNode'
    )

    launch_d.add_action(plate_camera_client)
    return launch_d
    