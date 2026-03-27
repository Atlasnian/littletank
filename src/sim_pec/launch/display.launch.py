import os
import xacro
from launch import LaunchDescription                   # launch文件的描述类
from launch_ros.actions import Node                    # 节点启动的描述类
from ament_index_python.packages import get_package_share_directory  # 用于获取包的路径
from launch.substitutions import Command    # 用于执行命令并获取输出结果

def generate_launch_description():

    # 获取rviz文件路径
    rviz_config_path =  os.path.join(	#拼接路径
        get_package_share_directory('sim_pec'),	#功能包路径
        'rviz',	#配置文件路径
        'display.rviz'	#配置文件
    )

    #获取xacro文件路径
    robot_model_path = os.path.join(
        get_package_share_directory('sim_pec'),
        'urdf',
        'pec_robot.urdf.xacro'
    )

    # 读取xacro文件并转换为urdf格式
    robot_description = xacro.process_file(robot_model_path).toxml()


    # 机器人状态发布节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True, 
                    }],
    )

    # 关节状态发布节点
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )

    # RViz2节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    return LaunchDescription([
        rviz_node,
        robot_state_publisher_node,
        joint_state_publisher_node
    ])

