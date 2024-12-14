import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, LogInfo, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    share_dir = get_package_share_directory('lio_sam')
    parameter_file = LaunchConfiguration('params_file')
    xacro_path = os.path.join(share_dir, 'config', 'robot.urdf.xacro')
    rviz_config_file = os.path.join(share_dir, 'config', 'rviz2.rviz')

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            share_dir, 'config', 'params.yaml'),
        description='FPath to the ROS2 parameters file to use.')
    

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

    # print("urdf_file_name : {}".format(xacro_path))

    return LaunchDescription([
        params_declare,
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments='0.0 0.0 0.0 0.0 0.0 0.0 map odom'.split(' '),
            parameters=[parameter_file],
            #output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro', ' ', xacro_path])
            }]
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_simpleGpsOdom',
            name='lio_sam_simpleGpsOdom',
            parameters=[parameter_file],
            arguments=['--ros-args', '--log-level', 'info'],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_imageProjection',
            name='lio_sam_imageProjection',
            parameters=[parameter_file],
            arguments=['--ros-args', '--log-level', 'info'],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_featureExtraction',
            name='lio_sam_featureExtraction',
            parameters=[parameter_file],
            arguments=['--ros-args', '--log-level', 'info'],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_mapOptimization',
            name='lio_sam_mapOptimization',
            parameters=[parameter_file],
            arguments=['--ros-args', '--log-level', 'info'],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        ),
        TimerAction(
                period=8.0,
                actions=[LogInfo(msg='Starting Octomap Server...'),
                        IncludeLaunchDescription(PythonLaunchDescriptionSource([octo_launch_file])),
                        ],
        ),
        TimerAction(
                period=10.0,
                actions=[LogInfo(msg='Starting Analysis...'),
                        IncludeLaunchDescription(PythonLaunchDescriptionSource([anal_launch_file])),
                        ],
        ),

    ])
