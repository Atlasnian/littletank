#必有项
import os
from launch import LaunchDescription                   # launch文件的描述类
from launch_ros.actions import Node                    # 节点启动的描述类

from ament_index_python.packages import get_package_share_directory  # 查询功能包路径的方法
from launch.substitutions import LaunchConfiguration    # 加载launch文件中的参数的描述类
from launch.actions import DeclareLaunchArgument       # 声明launch文件中的参数的描述类


def generate_launch_description():
    DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),
    return LaunchDescription([
        Node(
        package='ira_laser_tools',
        executable='laserscan_multi_merger',  # 可执行文件名
        name='laserscan_multi_merger',  # 必须与 YAML 文件中的节点名一致
        parameters=[
            {
                'destination_frame' : 'base_link',  # 目标坐标系
                'cloud_destination_topic' : '/merged_cloud',  # 输出点云话题
                'scan_destination_topic' : '/scan_multi',  # 输出激光扫描话题
                'laserscan_topics' : '/front_scan_filtered /back_scan_filtered',    # 激光扫描话题列表
                'angle_min' : -3.1415926,   # 最小角度
                'angle_max' : 3.1415926,    # 最大角度
                'angle_increment' : 0.00613592332229,    # 角度增量
                'time_increment' : 0.000055804,    # 时间增量
                'scan_time' : 0.1,    # 扫描时间
                'range_min' : 0.10,    # 最小范围
                'range_max' : 20.0,   # 最大范围
                'use_sim_time': LaunchConfiguration('use_sim_time')  # 使用仿真时间
            }
        ], 
        )
    ])








