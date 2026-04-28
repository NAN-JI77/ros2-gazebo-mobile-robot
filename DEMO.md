项目演示说明

演示目标

展示 ROS 2 + Gazebo 差速移动机器人完成 waypoint 巡航与 LaserScan 避障。

启动命令

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch turtle_tf_demo robot_task.launch.py
