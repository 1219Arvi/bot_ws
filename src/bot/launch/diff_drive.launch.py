import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler

from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # === File Paths ===
    bot_share = get_package_share_directory('bot')
    world_path = os.path.join(bot_share, 'worlds', 'world.sdf')
    xacro_file = os.path.join(bot_share, 'urdf', 'robot.xacro')
    slam_params = os.path.join(bot_share, 'config', 'mapper_params_online_async.yaml')

    # === Robot Description ===
    doc = xacro.process_file(xacro_file)
    robot_description = {'robot_description': doc.toxml()}

    # === Launch Gazebo ===
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_path,
            'verbose': 'true',
            'use_sim_time': 'true'
        }.items()
    )

    # === Nodes ===
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': True}],
        remappings=[('/joint_states', '/joint_states_merged')],
        output='screen'
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'base', '-x', '0.5', '-y', '0', '-z', '0'],
        output='screen'
    )

    joint_merger = Node(
        package='joint_states_merger',
        executable='joint_states_merger',
        name='joint_states_merger',
        parameters=[{
            'input_topics': ['/joint_states', '/joint_states_gui'],
            'output_topic': '/joint_states_merged',
            'use_sim_time': True
        }],
        output='screen'
    )

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        remappings=[('odom', '/diff_drive_base_controller/odom')],
        parameters=[
            slam_params,
            {
                'use_sim_time': True,
                'slam_toolbox.debug_logging': True,
                'message_filters.queue_size': 100  
            }
        ]
    )

    depth_to_scan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depth2scan',
        parameters=[{
            'output_frame': 'camera_depth_optical_link',
            'use_sim_time': True
        }],
        remappings=[
            ('depth', '/camera/depth/image_raw'),
            ('depth_camera_info', '/camera/depth/camera_info'),
            ('scan', '/scan')
        ],
        output='screen'
    )

    odom_relay = Node(
        package='topic_relay',
        executable='odom_relay_node',
        name='odom_relay',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # === Controller Loaders ===
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    load_diff_drive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'diff_drive_base_controller'],
        output='screen'
    )

    load_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'position_controller'],
        output='screen'
    )

    # === Controller Load Order ===
    on_spawn_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[load_joint_state_broadcaster]
        )
    )

    on_joint_state_loaded = RegisterEventHandler(
        OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_diff_drive_controller]
        )
    )

    on_diff_drive_loaded = RegisterEventHandler(
        OnProcessExit(
            target_action=load_diff_drive_controller,
            on_exit=[load_position_controller]
        )
    )

    # === Final Launch Description ===
    return LaunchDescription([
        gazebo,
        joint_merger,
        robot_state_publisher,
        spawn_entity,
        slam_toolbox,
        depth_to_scan,
        odom_relay,
        on_spawn_exit,
        on_joint_state_loaded,
        on_diff_drive_loaded
    ])
