from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_config_name = 'rog_map.yaml'

    declare_config_name_cmd = DeclareLaunchArgument(
        'config_name',
        default_value=default_config_name,
        description='config/ 下的 yaml 文件名'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    rog_map_node = Node(
        package='rog_map',
        executable='rog_map_node',
        name='rog_map_node',
        output='screen',
        parameters=[{
            'config_name': LaunchConfiguration('config_name'),
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
        }],
    )

    return LaunchDescription([
        declare_config_name_cmd,
        declare_use_sim_time_cmd,
        rog_map_node,
    ])
