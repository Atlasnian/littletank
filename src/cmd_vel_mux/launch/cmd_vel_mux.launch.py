#必有项
import os
from launch import LaunchDescription                   # launch文件的描述类
from launch_ros.actions import Node                    # 节点启动的描述类

from ament_index_python.packages import get_package_share_directory  # 查询功能包路径的方法
from launch.substitutions import LaunchConfiguration    # 加载launch文件中的参数的描述类
from launch.actions import DeclareLaunchArgument       # 声明launch文件中的参数的描述类

def generate_launch_description():
    #先创建launch文件内参数arg
    cmd_vel_out_arg=DeclareLaunchArgument(    #创建一个launch文件内参数arg，名为cmd_vel_out
        'cmd_vel_out',
        default_value='cmd_vel_stm'
    )
    config_locks_arg=DeclareLaunchArgument(   #创建一个launch文件内参数arg，名为config_locks
        'config_locks',
        default_value= os.path.join(      # 找到参数文件的完整路径
            get_package_share_directory('cmd_vel_mux'),
            'config',
            'twist_mux_locks.yaml'
            )
    )
    config_topics_arg=DeclareLaunchArgument(   #创建一个launch文件内参数arg，名为config_locks
        'config_topics',
        default_value= os.path.join(      # 找到参数文件的完整路径
            get_package_share_directory('cmd_vel_mux'),
            'config',
            'twist_mux_topics.yaml'
            )
    )

    #再获取arg的值，方便Node()使用
    cmd_vel_out = LaunchConfiguration('cmd_vel_out')    #这时cmd_vel_out值为cmd_vel_stm
    config_locks = LaunchConfiguration('config_locks')
    config_topics = LaunchConfiguration('config_topics')



    return LaunchDescription([
        cmd_vel_out_arg,    #将定义好的arg载入
        config_locks_arg,
        config_topics_arg,
        Node(
            package='twist_mux',   # 节点所在的功能包
            executable='twist_mux',  # 节点的可执行文件
            name='twist_mux',        # 节点名称
            output='screen',          # 输出到屏幕
            parameters=[
                config_locks,
                config_topics,
                # {"topics": {"collision": {"priority": 20}}}
            ],
            #parameters=[{"topics": {"collision": {"priority": 20}}}],
            remappings=[
                ('cmd_vel_out', cmd_vel_out),  #第一个from,第二个to
            ]
        ),
        Node(
            package='twist_mux',   # 节点所在的功能包
            executable='twist_marker',  # 节点的可执行文件
            name='twist_marker',        # 节点名称
            output='screen',          # 输出到屏幕
            remappings=[
                ('twist', cmd_vel_out),  #第一个from,第二个to
                ('marker', 'twist_marker'),
            ]
        )
    ])




