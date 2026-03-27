#必有项
import os
from launch import LaunchDescription                   # launch文件的描述类
from launch_ros.actions import Node                    # 节点启动的描述类


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy_control',
            executable='joy_control_node',
            name='joy_control',
            output='screen',
            parameters=[{
                'max_linear_speed': 0.5,  #最大速度0.3m/s
                'max_rotation_speed': 0.4,
                'accel_linear_speed_per': 0.01, #1秒内要加速到最大值：accel_linear_speed_per*30 = max_linear_speed
            }],
            # remappings=[('/cmd_vel', '/cmd_vel_stm')]
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            # parameters=[{
            #     'device_name': '/dev/input/js1', # 设备路径，默认js0
            # }],
        ),
    ])