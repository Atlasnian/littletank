#必有项
import os
from launch import LaunchDescription                   # launch文件的描述类
from launch_ros.actions import Node                    # 节点启动的描述类

from launch.actions import IncludeLaunchDescription # 引入其他launch文件
from launch_ros.substitutions import FindPackageShare # 查找ROS2包的路径
from launch.launch_description_sources import PythonLaunchDescriptionSource #声明源的类型
from launch.substitutions import PathJoinSubstitution   #路径拼接
from launch.actions import DeclareLaunchArgument # 声明launch文件参数
from launch.substitutions import EnvironmentVariable #加载环境变量
from launch.conditions import IfCondition  #引入条件判断
from launch.substitutions import LaunchConfiguration  #获取launch文件内参数
from launch.substitutions import PythonExpression  # 表达式，用于条件判定
from launch.actions import TimerAction  #延迟
from ament_index_python.packages import get_package_share_directory #获取包路径


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),


        # 引入joy_control的launch文件
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('joy_control'),
                    'launch/joycontrol.launch.py'
                ]), 
            ])
        ),

        #启动gazebo仿真环境
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('sim_pec'),
                    'launch/gazebo_sim.launch.py'
                ]),
            ]),
        ),

        #front laser filter
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='front_laser_filter',
            parameters=[
                os.path.join(get_package_share_directory('bring_up'), 'config', 'front_laser_filter.yaml'),
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }
            ],
            remappings=[
                ('scan', 'front_scan'),
                ('scan_filtered', 'front_scan_filtered')
            ],
            output='screen',
        ),

        #back laser filter
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='back_laser_filter',
            parameters=[
                os.path.join(get_package_share_directory('bring_up'), 'config', 'back_laser_filter.yaml'),
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }
            ],
            remappings=[
                ('scan', 'back_scan'),
                ('scan_filtered', 'back_scan_filtered')
            ],
            output='screen',
        ),

        #laser merge
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ira_laser_tools'),
                    'launch/laserscan_multi_merger.launch.py'
                ]),
            ])
        ),


    ])