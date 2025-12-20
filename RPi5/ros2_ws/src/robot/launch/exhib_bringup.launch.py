#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('robot')
    default_model_path = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    test_yaml = os.path.join(pkg_share, 'config', 'nav_exhib_lidar.yaml')

    use_lifecycle_manager = LaunchConfiguration("use_lifecycle_manager")
    use_sim_time = LaunchConfiguration("use_sim_time")
    lidar_port = LaunchConfiguration('usb_port', default='/dev/ttyUSB0')

    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('sllidar_ros2'),
                'launch',
                'view_sllidar_a1_launch.py'
            )
        ),
        launch_arguments={
            'serial_port': lidar_port,
            'frame_id': 'lidar_frame',
            'use_sim_time': use_sim_time
        }.items()
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', LaunchConfiguration('model')]),
                value_type=str
            ),
            'use_sim_time': use_sim_time,
            'publish_frequency': 30.0,
            'publish_static_joints': True
        }],
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[default_model_path],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    zynq_packet_node = Node(
        package='robot',
        executable='zynq_node',
        name='zynq_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    merge_node = Node(
        package='robot',
        executable='merge_lidar_ultrasonic_node',
        name='merge_lidar_ultrasonic_node',
        output='screen',
        parameters=[{'use_ultrasonic': True, 'use_sim_time': use_sim_time}],
        remappings=[
            ('/echo_front', '/es0'),
            ('/echo_right', '/es2'),
            ('/echo_back', '/es3'),
            ('/echo_left', '/es5'),
        ],
    )

    yolo_node = Node(
        package='robot',
        executable='person_receiver_exhib',
        name='person_receiver',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # 1. Velocity Smoother (Remapping 제거, YAML 설정 사용)
    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[test_yaml],
        remappings=[
            # 노드가 듣고 있는 이름(cmd_vel) -> 우리가 보내는 이름(cmd_vel_raw)
            ('cmd_vel', 'cmd_vel_raw'), 
            # 노드가 뱉는 이름(cmd_vel_smoothed) -> 우리가 원하는 이름(cmd_vel_smoothed)
            ('smoothed_cmd_vel', 'cmd_vel_smoothed'),
        ],
    )

    # 2. Collision Monitor (Remapping 제거, YAML 설정 사용)
    collision_monitor_node = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[test_yaml],
        # YAML 파일에 topic 설정이 있으므로 remapping 불필요
    )
    
    gui = Node(
        package='robot',
        executable='mode_gui',
        name='mode_gui',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # 3. Lifecycle Manager 추가 (가장 중요한 부분!)
    # velocity_smoother와 collision_monitor를 활성화(Active) 시켜줍니다.
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_costmap_filters',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},  # 자동으로 노드들을 시작 상태로 전환
            {'node_names': ['velocity_smoother', 'collision_monitor']} # 관리할 노드 이름 목록
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='model',
            default_value=default_model_path,
            description='Absolute path to robot model file'
        ),

        DeclareLaunchArgument(
            name='usb_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for LiDAR'
        ),

        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        DeclareLaunchArgument(
            'use_lifecycle_manager',
            default_value='false',
            description='Enable lifecycle manager bond connection'
        ),

        joint_state_publisher_node,
        robot_state_publisher_node,
        zynq_packet_node,
        merge_node,
        yolo_node,
        velocity_smoother_node,
        collision_monitor_node,
        lifecycle_manager_node, # 리스트에 매니저 추가
        sllidar_launch,
        gui,
    ])
