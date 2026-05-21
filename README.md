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
```



4. SLAM 建图与地图保存

本节记录 TurtleBot3 Burger + Cartographer 在 Gazebo Classic 中建图的稳定启动流程。

Gazebo 启动命令：

```bash
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_DATABASE_URI=""
export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:/opt/ros/humble/share/turtlebot3_gazebo/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

Cartographer 启动命令：

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

teleop 控制命令：

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard
```

RViz 设置：

- Global Options -> Fixed Frame: `map`
- Map -> Topic: `/map`
- Map -> Reliability: `Reliable`
- Map -> Durability: `Transient Local`
- LaserScan -> Topic: `/scan`
- 可打开 TF 和 RobotModel 辅助检查坐标树。

地图保存命令：

```bash
mkdir -p ~/ros2_ws/maps
source /opt/ros/humble/setup.bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/maps/turtlebot3_slam_map
```

保存成功后会生成：

```text
~/ros2_ws/maps/turtlebot3_slam_map.pgm
~/ros2_ws/maps/turtlebot3_slam_map.yaml
```

Nav2 加载已保存地图：

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/ros2_ws/maps/turtlebot3_slam_map.yaml
```

Nav2 启动后，在 RViz 中按下面顺序操作：

1. Fixed Frame 设置为 `map`。
2. 确认 Map 话题为 `/map`，并能显示 `turtlebot3_slam_map.yaml` 对应地图。
3. 使用 `2D Pose Estimate` 在地图白色可通行区域给 burger 设置初始位置和朝向。
4. 使用 `Nav2 Goal` 在白色可通行区域设置目标点，观察路径线和小车运动。

本次保存的地图范围大约是 `x=[-1.20, 3.45]`、`y=[-2.38, 3.37]`。初始位姿必须落在这个地图范围内。之前把初始位姿设到 `x=-1.95, y=-0.45` 时会触发 costmap 的 `Robot is out of bounds`，Nav2 不会规划。实际测试中，初始位姿设在地图内部后，目标点 `x=0.775, y=-0.055` 可以成功导航，终端出现 `Goal succeeded`，小车里程计位置也发生移动。

更详细的排查记录见 `SLAM_NOTES.md`。
