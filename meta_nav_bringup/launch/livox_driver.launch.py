from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindPackageShare
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals, IfCondition

ARGUMENTS = [
    DeclareLaunchArgument(
        'user_config_path',
        default_value='',
        description='Path to user configuration file'),
]


def generate_launch_description():
    user_config_path = LaunchConfiguration('user_config_path')

    # 0-PointCloud2Msg(PointXYZRTL), 1-LivoxCustomMsg, 2-PclPxyziMsg, 3-LivoxImuMsg, 4-AllMsg
    xfer_format = 4
    multi_topic = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
    data_src = 0    # 0-lidar, others-Invalid data src
    publish_freq = 10.0  # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
    output_type = 0
    frame_id = 'livox_frame'
    lvx_file_path = '/home/livox/livox_test.lvx'
    cmdline_bd_code = 'livox0000000001'

    livox_ros2_params = [
        {"xfer_format": xfer_format},
        {"multi_topic": multi_topic},
        {"data_src": data_src},
        {"publish_freq": publish_freq},
        {"output_data_type": output_type},
        {"frame_id": frame_id},
        {"lvx_file_path": lvx_file_path},
        {"user_config_path": user_config_path},
        {"cmdline_input_bd_code": cmdline_bd_code}
    ]

    livox_ros_driver2 = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='both',
        emulate_tty=True,
        parameters=livox_ros2_params
    )

    return LaunchDescription([livox_ros_driver2])
