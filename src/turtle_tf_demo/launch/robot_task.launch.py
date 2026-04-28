from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    # my_robot_description 的 Gazebo 启动文件
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('my_robot_description'),
                'launch',
                'gazebo_spawn.launch.py'
            )
        )
    )

    # cmd_vel bridge：ROS <-> Gazebo 控制速度
    cmd_vel_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        name='cmd_vel_bridge',
        arguments=[
            '/model/simple_robot/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist'
        ],
        output='screen'
    )

    # odometry bridge：Gazebo -> ROS 里程计反馈
    odom_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        name='odom_bridge',
        arguments=[
            '/model/simple_robot/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry'
        ],
        output='screen'
    )

    # scan bridge：Gazebo -> ROS 激光雷达
    scan_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        name='scan_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan'
        ],
        output='screen'
    )

    # relay：把 /cmd_vel 转发到 /model/simple_robot/cmd_vel
    cmd_vel_relay = Node(
        package='turtle_tf_demo',
        executable='cmd_vel_relay',
        name='cmd_vel_relay',
        output='screen'
    )

    # 状态机版 waypoint 巡航 + 避障节点
    waypoint_avoid_sm = Node(
        package='turtle_tf_demo',
        executable='waypoint_avoid_sm',
        name='waypoint_avoid_sm',
        output='screen'
    )

    return LaunchDescription([
        # 先启动 Gazebo 和小车
        robot_launch,

        # 延迟 3 秒再启动 bridge，避免 Gazebo topic 还没创建
        TimerAction(
            period=3.0,
            actions=[
                cmd_vel_bridge,
                odom_bridge,
                scan_bridge,
            ]
        ),

        # 延迟 5 秒启动 relay 和控制节点
        TimerAction(
            period=5.0,
            actions=[
                cmd_vel_relay,
                waypoint_avoid_sm,
            ]
        ),
    ])
