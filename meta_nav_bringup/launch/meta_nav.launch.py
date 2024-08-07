from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals, IfCondition
from launch_ros.substitutions import FindPackageShare

ARGUMENTS = [
    DeclareLaunchArgument(
        'enable_simulation',
        default_value='false',
        description='If true, the simulation will be started'),
    DeclareLaunchArgument(
        'enable_lio_rviz',
        default_value='false',
        description='If true, the RViz for LIO will be started'),
]


def generate_launch_description():
    enable_simulation = LaunchConfiguration('enable_simulation')
    enable_lio_rviz = LaunchConfiguration('enable_lio_rviz')

    # Livox Driver
    livox_config_path = PathJoinSubstitution(
        [FindPackageShare('meta_nav_bringup'), 'config', 'mid360_params.json'])
    livox_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('meta_nav_bringup'), 'launch', 'livox_driver.launch.py'])
        ),
        launch_arguments=[
            ('user_config_path', livox_config_path),
        ]
    )

    # Point-LIO
    pointlio_mid360_params = PathJoinSubstitution(
        [FindPackageShare('meta_nav_bringup'), 'config', 'pointlio_mid360.yaml'])
    pointlio_rviz_cfg_dir = PathJoinSubstitution(
        [FindPackageShare('meta_nav_bringup'), 'rviz', 'pointlio.rviz'])
    point_lio = GroupAction(
        actions=[
            Node(
                package='point_lio',
                executable='pointlio_mapping',
                name='laserMapping',
                parameters=[
                    pointlio_mid360_params,
                    {'use_sim_time': enable_simulation,
                     'use_imu_as_input': False,  # Change to True to use IMU as input of Point-LIO
                     'prop_at_freq_of_imu': True,
                     'check_satu': True,
                     'init_map_size': 10,
                     'point_filter_num': 3,  # Options: 1, 3
                     'space_down_sample': True,
                     'filter_size_surf': 0.5,  # Options: 0.5, 0.3, 0.2, 0.15, 0.1
                     'filter_size_map': 0.5,  # Options: 0.5, 0.3, 0.15, 0.1
                     'ivox_nearby_type': 6,   # Options: 0, 6, 18, 26
                     'runtime_pos_log_enable': False}
                ],
                output='both',
                emulate_tty=True,
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', pointlio_rviz_cfg_dir],
                condition=IfCondition(enable_lio_rviz),
            )
        ]
    )

    return LaunchDescription([
        *ARGUMENTS,
        livox_driver,
        point_lio,
    ])
