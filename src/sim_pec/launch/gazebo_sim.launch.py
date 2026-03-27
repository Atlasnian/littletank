import os
import xacro
from launch import LaunchDescription                   # launch文件的描述类
from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch_ros.actions import Node                    # 节点启动的描述类
from ament_index_python.packages import get_package_share_directory  # 用于获取包的路径
from launch.substitutions import Command    # 用于执行命令并获取输出结果
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    # 获取rviz文件路径
    rviz_config_path =  os.path.join(	#拼接路径
        get_package_share_directory('sim_pec'),	#功能包路径
        'rviz',	#配置文件路径
        'display.rviz'	#配置文件
    )

    # 获取world文件路径
    world_file_path = os.path.join(
        get_package_share_directory('sim_pec'),
        'world',
        'test.sdf'
    )
    print("World file path:", world_file_path)

    #获取xacro文件路径
    robot_model_path = os.path.join(
        get_package_share_directory('sim_pec'),
        'urdf',
        'pec_robot.urdf.xacro'
    )

    #获取bridge config文件路径
    config_file_path = os.path.join(
    get_package_share_directory('sim_pec'),
    'config',
    'bridge_config.yaml'  #创建一个config.yaml文件并载入
)

    # 读取xacro文件并转换为urdf格式
    robot_description = xacro.process_file(robot_model_path).toxml()

    # 机器人状态发布节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
                        'robot_description': robot_description,
                        'use_sim_time': True, 
                    }],
    )

    # 关节状态发布节点
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}],
    )

    # 启动gazebo仿真环境
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 
                        'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': world_file_path}.items()
    )

    # 请求Gazebo加载机器人
    spawn_model_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'pec_robot',
            '-string', robot_description,
            '-allow_renaming', 'false',
            '-x', '-8.5', '-y', '12.34', '-z', '0.002'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 桥接Gazebo和ROS2的话题数据
    gz_ros_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': config_file_path,
            'use_sim_time': True,
        }],
        output='screen'
    )

    # 桥接camera和depth数据
    # image_bridge_node=Node(
    #     package='ros_gz_image',
    #     executable='image_bridge',
    #     arguments=[
    #         '/camera',
    #     ],
    #     parameters=[{'use_sim_time': True}],
    #     output='screen'
    # )

    # RViz2节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        gazebo_sim,
        spawn_model_node,
        gz_ros_bridge,
        # image_bridge_node,
        rviz_node,
    ])

