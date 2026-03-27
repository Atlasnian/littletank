from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value = 'True')
    pkg_share = FindPackageShare('cartographer_ros').find('cartographer_ros')

    cartographer_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_node',
        parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments = [
            '-configuration_directory', FindPackageShare('cartographer_ros').find('cartographer_ros') + '/configuration_files',
            '-configuration_basename', 'backpack_2d.lua'],
        remappings = [
            ('echoes', 'horizontal_laser_2d'),
            ('scan','/scan_multi'),
            ('imu','/imu_data'),
            ('odom', '/odom')],
        output = 'screen'
        )

    cartographer_occupancy_grid_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_occupancy_grid_node',
        parameters = [
            {'use_sim_time': True},
            {'resolution': 0.05}],
        )

    return LaunchDescription([
        use_sim_time_arg,
        cartographer_node,
        cartographer_occupancy_grid_node,
    ])