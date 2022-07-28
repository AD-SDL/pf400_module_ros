from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            namespace='turtlesim2',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',
            remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ]
        )
    ])


    # <!-- <launch>

#   <arg name="ip" default="192.168.50.50" />

#   <param name="TCS_IP" value="$(arg ip)" />
#   <param name="robot_description" command="$(find xacro)/xacro.py $(find pa_viz_pf3400)/urdf/PF3400SX.urdf" />
#   <param name="use_gui" value="true"/>

#   <node name="PA_Viz_PF400" pkg="pa_viz_pf400" type="TCSJointPub.py" />
#   <node name="robot_state_publisher" pkg="robot_state_publisher" type="state_publisher" />
#   <node name="rviz" pkg="rviz" type="rviz" args="-d $(find pa_viz_pf3400)/rviz/StandardSettings.rviz" required="true" />

# </launch> -->
