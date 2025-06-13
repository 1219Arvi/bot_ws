from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    # === Process the Xacro file ===
    urdf_file = os.path.join(
        get_package_share_directory('bot'),
        'urdf',
        'robot.xacro'
    )
    doc = xacro.process_file(urdf_file)
    robot_description_config = {'robot_description': doc.toxml()}

    return LaunchDescription([

        # Joint States Merger
        Node(
            package='joint_states_merger',
            executable='joint_states_merger',
            name='joint_states_merger',
            output='screen',
            parameters=[{
                'input_topics': ['/joint_states', '/joint_states_gui'],
                'output_topic': '/joint_states_merged',
                'use_sim_time': True
            }]
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description_config, {'use_sim_time': True}],
            remappings=[('/joint_states', '/joint_states_merged')]
        ),

        # GUI for manually manipulating joints
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            remappings=[('/joint_states', '/joint_states_gui')],
            parameters=[{'use_sim_time': True}]
        ),

        # RViz Viewer
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': True}],
        )
    ])
