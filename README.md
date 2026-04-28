ROS 2 差速移动机器人仿真与巡航避障系统

1. 项目简介

本项目基于 ROS 2 Humble、Gazebo、RViz、URDF/xacro 搭建了一个差速移动机器人仿真系统，实现了机器人建模、Gazebo 仿真、`/cmd_vel` 控制、odometry 闭环反馈、waypoint 巡航、LaserScan 激光雷达感知以及基于状态机的简单避障功能。

项目最终实现效果：

- 在 Gazebo 中生成差速小车模型
- 通过 `/cmd_vel` 控制小车运动
- 通过 odometry 获取小车位姿反馈
- 实现闭环距离控制、闭环角度控制、目标点控制、多目标点巡航
- 集成 LaserScan 激光雷达并桥接到 ROS 2
- 实现 waypoint 巡航 + 激光雷达避障融合
- 使用 launch 文件一键启动完整系统



2. 技术栈

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo / Ignition Gazebo
- RViz2
- Python
- URDF / xacro
- robot_state_publisher
- ros_ign_bridge / ros_gz_bridge
- nav_msgs / sensor_msgs / geometry_msgs
- colcon



3. 项目结构

```text
ros2_ws/
├── src/
│   ├── my_robot_description/
│   │   ├── launch/
│   │   │   └── gazebo_spawn.launch.py
│   │   ├── urdf/
│   │   │   └── simple_robot.urdf.xacro
│   │   ├── worlds/
│   │   │   └── empty_with_sensors.sdf
│   │   ├── package.xml
│   │   └── setup.py
│   │
│   └── turtle_tf_demo/
│       ├── launch/
│       │   ├── follow_waypoints_param.launch.py
│       │   └── robot_task.launch.py
│       ├── turtle_tf_demo/
│       │   ├── cmd_vel_relay.py
│       │   ├── drive_with_odom.py
│       │   ├── turn_with_odom.py
│       │   ├── square_with_odom.py
│       │   ├── go_to_goal.py
│       │   ├── follow_waypoints.py
│       │   ├── follow_waypoints_param.py
│       │   ├── go_to_goal_smooth.py
│       │   ├── follow_waypoints_smooth.py
│       │   ├── obstacle_avoid.py
│       │   ├── waypoint_avoid.py
│       │   └── waypoint_avoid_sm.py
│       ├── package.xml
│       └── setup.py
└── README.md
