from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 获取 my_robot_description 包的安装路径
    pkg_path = get_package_share_directory('my_robot_description')

    # xacro 机器人模型文件
    xacro_file = os.path.join(
        pkg_path,
        'urdf',
        'simple_robot.urdf.xacro'
    )

    # 带传感器系统的 Gazebo world 文件
    world_file = os.path.join(
        pkg_path,
        'worlds',
        'empty_with_sensors.sdf'
    )

    # 把 xacro 转成 robot_description
    robot_description = Command([
        'xacro ',
        xacro_file
    ])

    # robot_state_publisher 发布机器人模型和 TF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {
                'robot_description': robot_description
            }
        ],
        output='screen'
    )

    # 启动 Gazebo，并加载我们自己的 world 文件
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}'
        }.items()
    )

    # 把 robot_description 里的机器人生成到 Gazebo 中
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'simple_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.2'
        ],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        gz_sim,
        spawn_robot
    ])
