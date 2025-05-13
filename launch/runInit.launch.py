import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, LogInfo, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler


def generate_launch_description():
    share_dir = get_package_share_directory('lio_sam')
    parameter_file = LaunchConfiguration('params_file')
    xacro_path = os.path.join(share_dir, 'config', 'robot.urdf.xacro')
    rviz_config_file = os.path.join(share_dir, 'config', 'rviz2.rviz')

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(share_dir, 'config', 'params.yaml'),
        description='Path to the ROS2 parameters file to use.'
    )

    # Launch files to be included later
    octo_launch_file = os.path.join(
        get_package_share_directory('octomap_server2'),
        'launch',
        'octomap_server_launch.py'
    )
    
    anal_launch_file = os.path.join(
        get_package_share_directory('analysis'),
        'launch',
        'analysis.launch.py'
    )

    # Nodes that must launch first
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments='0.0 0.0 0.0 0.0 0.0 0.0 map odom'.split(' '),
        parameters=[parameter_file],
        #output='screen'
    )

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro', ' ', xacro_path])
        }]
    )

    simple_gps_odom = Node(
        package='lio_sam',
        executable='lio_sam_simpleGpsOdom',
        name='lio_sam_simpleGpsOdom',
        parameters=[parameter_file],
        arguments=['--ros-args', '--log-level', 'info'],
        output='screen'
    )

    wait_for_origin_init_node = Node(
        package='lio_sam',
        executable='lio_sam_waitForGpsOrigin',
        name='lio_sam_waitForGpsOrigin',
        output='screen'
    )

    image_projection = Node(
        package='lio_sam',
        executable='lio_sam_imageProjection',
        name='lio_sam_imageProjection',
        parameters=[parameter_file],
        arguments=['--ros-args', '--log-level', 'info'],
        output='screen'
    )

    feature_extraction = Node(
        package='lio_sam',
        executable='lio_sam_featureExtraction',
        name='lio_sam_featureExtraction',
        parameters=[parameter_file],
        arguments=['--ros-args', '--log-level', 'info'],
        output='screen'
    )

    map_optimization = Node(
        package='lio_sam',
        executable='lio_sam_mapOptimization',
        name='lio_sam_mapOptimization',
        parameters=[parameter_file],
        arguments=['--ros-args', '--log-level', 'info'],
        output='screen'
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    start_octomap = TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg='Starting Octomap Server...'),
            IncludeLaunchDescription(PythonLaunchDescriptionSource([octo_launch_file])),
        ]
    )

    start_analysis = TimerAction(
            period=10.0,
            actions=[
                LogInfo(msg='Starting Analysis...'),
                IncludeLaunchDescription(PythonLaunchDescriptionSource([anal_launch_file])),
            ]
    )

    wait_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_origin_init_node,
            on_exit=[
                LogInfo(msg='GPS origin initialised. Launching remaining nodes...'),
                image_projection,
                feature_extraction,
                map_optimization,
                rviz2_node,
                start_octomap,
                # start_analysis
            ]
        )
    )

    # Create the final launch description.
    return LaunchDescription([
        params_declare,
        # Launch initial nodes.
        static_tf,
        robot_state_pub,
        simple_gps_odom,
        # Wait for service to become available.
        wait_for_origin_init_node,
        wait_handler
    ])
