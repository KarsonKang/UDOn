from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value = 'false')

    # pkg_share = FindPackageShare('cartographer_ros').find('cartographer_ros')

    cartographer_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_node',
        arguments = [
            '-configuration_directory', FindPackageShare('udon_controller').find('udon_controller') + '/config',
            '-configuration_basename', 'UDOn_backpack_2d.lua'],
        remappings = [
            ('odom', 'odometry/filtered'),
            ('scan', 'scan')
            # ('echoes', 'scan')
            ],
        output = 'screen',
    )

    occupancy_grid_node = Node(
        package = 'cartographer_ros',
        executable = 'occupancy_grid_node',
        parameters = [
            {'use_sim_time': False},
            {'resolution': 0.05}],
    )

    return LaunchDescription([
        use_sim_time_arg,
        cartographer_node,
        occupancy_grid_node,
    ])

    