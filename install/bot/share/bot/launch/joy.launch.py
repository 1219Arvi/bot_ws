from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_joy',
            parameters=[{
                'axis_linear': 1,
                'scale_linear': 0.5,
                'axis_angular': 2,
                'scale_angular': 1.0,
                'enable_button': 5,
                'publish_stamped_twist': True
            }],
            remappings=[
                ('/cmd_vel', '/diff_drive_base_controller/cmd_vel')  # Optional, can remap if needed
            ]
        )
    ])
