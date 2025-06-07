import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # Load the world file
    world_file_name = 'obstacles.world'
    world_path = os.path.join(get_package_share_directory('bot'), 'worlds', world_file_name)

    # Include Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'world': world_path,
            'verbose': 'true',
            'use_sim_time': 'true',
            'debug': 'false'
        }.items()
    )

    # Load robot description from Xacro
    xacro_file = os.path.join(
        get_package_share_directory('bot'),
        'urdf',
        'robot.xacro'
    )
    doc = xacro.process_file(xacro_file)
    robot_description = {'robot_description': doc.toxml()}

    # Start robot_state_publisher (remapped to merged joint states)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
        remappings=[('/joint_states', '/joint_states_merged')]
    )

    # Spawn robot into Gazebo from robot_description
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'base'],
        output='screen'
    )

    # Load joint state broadcaster after robot is spawned
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    # Load diff drive controller after joint state broadcaster
    load_diff_drive_base_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'diff_drive_base_controller'],
        output='screen'
    )

    # Static joint state publisher GUI (remaps to /joint_states_static)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        remappings=[
            ('/joint_states', '/joint_states_static')
        ],
        parameters=[{'use_sim_time': True}]
    )

    # Joint states merger node
    joint_states_merger_node = Node(
        package='joint_states_merger',
        executable='joint_states_merger',
        name='joint_states_merger',
        output='screen',
        parameters=[{
            'input_topics': ['/joint_states', '/joint_states_static'],
            'output_topic': '/joint_states_merged',
            'use_sim_time': True
        }]
    )

    return LaunchDescription([
        gazebo,
        joint_state_publisher_gui_node,
        joint_states_merger_node,
        node_robot_state_publisher,
        spawn_entity,

        # Load controllers in correct sequence
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_diff_drive_base_controller]
            )
        ),
        Node(
            package='image_flip_node',
            executable='image_flip',
            name='flip_depth_image',
            output='screen'
        )
    ])
