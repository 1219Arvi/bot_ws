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

    slam_params = os.path.join(
        get_package_share_directory('bot'),
        'config',
        'mapper_params_online_async.yaml'
    )

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

    # Static joint state publisher GUI (remaps to /joint_states_gui)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        remappings=[
            ('/joint_states', '/joint_states_gui')
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
            'input_topics': ['/joint_states', '/joint_states_gui'],
            'output_topic': '/joint_states_merged',
            'use_sim_time': True
        }]
    )
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        remappings=[
        ('odom', '/diff_drive_base_controller/odom'),
          ],
        parameters=[slam_params, {'use_sim_time': True}],
)


    return LaunchDescription([
        gazebo,
        joint_state_publisher_gui_node,
        joint_states_merger_node,
        node_robot_state_publisher,
        spawn_entity,
        slam_toolbox_node,

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
        ),
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depth2scan',
            output='screen',
            parameters=[{
                'output_frame_id': 'camera_link_optical' ,
                'use_sim_time': True 
            }],
            remappings=[
            ('depth', '/flip_depth/image'),  # From your image_flip_node
            ('depth_camera_info', '/camera/depth/camera_info'),
            ('scan', '/scan')  # slam_toolbox listens here
        ]
        ),
        Node(
            package='topic_relay',
            executable='odom_relay_node',
            name='odom_relay',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='vision_to_realsense_broadcaster',
        #     arguments=[
        #         '0', '0', '0', '0', '0', '0',
        #         'vision_module_link_1', 'realsense_link'
        #     ],
        #     output='screen',
        #     parameters=[],
        #     # Important: ensure the node doesn't immediately exit!
        #     # Use 'respawn=True' if needed, but better:
        #     emulate_tty=True
        # ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='realsense_to_optical_broadcaster',
        #     arguments=[
        #         '0', '0', '0', '0', '0', '0',
        #         'realsense_link', 'camera_link_optical'
        #     ],
        #     output='screen',
        #     emulate_tty=True
        # )

   ])
