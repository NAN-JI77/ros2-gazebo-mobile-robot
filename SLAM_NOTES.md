# TurtleBot3 Cartographer SLAM Notes

本文记录 Ubuntu 22.04 + ROS 2 Humble + Gazebo Classic 下 TurtleBot3 Burger 使用 Cartographer 建图的稳定启动流程、检查方法、RViz 设置和地图保存命令。

## 1. 干净启动流程

建议先确认没有旧的 SLAM、Nav2、RViz、Gazebo 进程残留。残留节点可能重复发布 `/tf` 或占用已有 Gazebo 实体，导致 `/map` 不稳定。

终端 1：启动 Gazebo TurtleBot3 world。

```bash
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_DATABASE_URI=""
export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:/opt/ros/humble/share/turtlebot3_gazebo/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

启动后确认 Gazebo 日志中出现：

```text
Spawn status: SpawnEntity: Successfully spawned entity [burger]
Publishing odom transforms between [odom] and [base_footprint]
```

终端 2：启动 Cartographer。

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

终端 3：键盘控制小车。

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard
```

## 2. 建图状态检查

常用检查命令：

```bash
ros2 node list
ros2 topic list
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic hz /clock
ros2 topic echo /map --once
ros2 topic echo /submap_list --once
ros2 run tf2_ros tf2_echo map odom
```

正常现象：

- `/scan` 有频率，通常约 4-5 Hz。
- `/odom` 有频率，通常约 20-30 Hz。
- `/clock` 有输出，Cartographer 使用 `use_sim_time:=True`。
- `/map` 类型为 `nav_msgs/msg/OccupancyGrid`，可以 echo 到 `header`、`info`、`data`。
- `map -> odom` 可以查到，z 应接近 0，不应出现几万、几十万或百万级数值。

## 3. 常见问题

### `/map` 不稳定或 RViz Map 报 Warning/Error

先检查 `/odom` 的 z：

```bash
ros2 topic echo /odom --once --field pose.pose.position
```

如果 z 是几万、几十万或百万级，通常是 Gazebo 物理世界异常，例如地面没有正确加载、旧 Gazebo 进程残留、重复实体 `burger` 没清干净。先干净关闭旧 Gazebo/Cartographer/RViz/teleop，再重新启动。

### `map -> odom` 不存在

检查 Cartographer 是否真的收到 `/scan` 和 `/odom`：

```bash
ros2 topic info /scan --verbose
ros2 topic info /odom --verbose
ros2 param get /cartographer_node use_sim_time
```

TurtleBot3 Burger 默认 Cartographer 配置通常为：

```text
tracking_frame = "imu_link"
published_frame = "odom"
odom_frame = "odom"
provide_odom_frame = false
use_odometry = true
```

如果 `/scan` 的 `frame_id` 是 `base_scan`，需要 TF 树中存在 `base_footprint -> base_link -> base_scan`。

### Gazebo 模型或地面加载问题

如果 Gazebo 日志出现 `Unable to find uri[model://ground_plane]` 或 `Unable to find uri[model://sun]`，建议显式设置：

```bash
export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:/opt/ros/humble/share/turtlebot3_gazebo/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}
```

### RViz OpenGL/GLSL 报错

如果 ROS graph、TF 和 `/map` 都正常，但 RViz Map 显示仍异常，可尝试软件渲染：

```bash
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

## 4. RViz 设置

- Global Options -> Fixed Frame: `map`
- Map -> Topic: `/map`
- Map -> Reliability: `Reliable`
- Map -> Durability: `Transient Local`
- LaserScan -> Topic: `/scan`
- 建议打开 TF 和 RobotModel，辅助检查 TF 树是否正常。

## 5. 保存地图

建图完成并确认 `/map` 正常后，创建地图目录并保存：

```bash
mkdir -p ~/ros2_ws/maps
source /opt/ros/humble/setup.bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/maps/turtlebot3_slam_map
```

成功后会生成：

```text
~/ros2_ws/maps/turtlebot3_slam_map.pgm
~/ros2_ws/maps/turtlebot3_slam_map.yaml
```

后续使用 Nav2 或 map_server 加载地图时，通常传入 `.yaml` 文件路径。

## 6. Nav2 加载地图并自主导航

建图和地图保存完成后，Nav2 使用保存出来的 YAML 文件加载地图：

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/ros2_ws/maps/turtlebot3_slam_map.yaml
```

启动后检查核心节点生命周期：

```bash
ros2 lifecycle get /amcl
ros2 lifecycle get /map_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
```

正常应为 `active [3]`。本次成功测试时，`/amcl`、`/map_server`、`/planner_server`、`/controller_server`、`/bt_navigator` 都已进入 `active [3]`。

RViz 操作顺序：

1. Global Options -> Fixed Frame 设置为 `map`。
2. Map 显示 `/map`，地图来自 `~/ros2_ws/maps/turtlebot3_slam_map.yaml`。
3. 使用 `2D Pose Estimate` 设置初始位姿。箭头位置和方向要对应 Gazebo 中 burger 的实际位置和朝向。
4. 使用 `Nav2 Goal` 设置导航目标。目标点应放在白色可通行区域，避开黑色障碍、灰色未知区和膨胀区。

本次地图 YAML 内容：

```yaml
image: turtlebot3_slam_map.pgm
mode: trinary
resolution: 0.05
origin: [-1.2, -2.38, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

地图尺寸对应的有效范围大约是 `x=[-1.20, 3.45]`、`y=[-2.38, 3.37]`。如果 `2D Pose Estimate` 点到范围外，Nav2 会在 costmap 里报 `Robot is out of bounds`，即使节点全是 active，也不会规划和移动。之前测试的 `x=-1.95, y=-0.45` 就在地图范围外，因此点击 Nav2 Goal 后小车不动。

本次成功导航验证：

```text
初始位姿：地图内部白色区域，约 x=-0.025, y=-0.005
测试目标：x=0.775, y=-0.055
结果：Goal accepted -> Reached the goal -> Goal succeeded
现象：/cmd_vel 有速度输出，Gazebo 中 burger 向目标移动，/odom 位置发生变化
```

也可以用命令行直接发送一个短距离目标做后端验证：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 0.775, y: -0.055, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}" --feedback
```

如果 RViz 点击目标后不动，按顺序检查：

```bash
ros2 topic hz /odom
ros2 topic hz /scan
ros2 topic info /map --verbose
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
```

常见原因：

- 没有先用 `2D Pose Estimate` 正确初始化 AMCL。
- 初始位姿在地图范围外，costmap 报 `Robot is out of bounds`。
- 目标点在障碍物、未知区域或膨胀区。
- 点的是 RViz 旧的 `2D Goal Pose`，不是 Nav2 的 `Nav2 Goal`。
- Nav2 lifecycle 没有进入 `active [3]`。
- `/odom`、`/scan`、`map -> odom`、`odom -> base_footprint` 任意一个异常。
