# maze_wall_pid：合并迷宫 PID 寻左墙

雷达墙点拟合、法向偏移、曲率前馈、朝向修正与横向 PID；使用双墙几何辅助处理合并迷宫中的弯道。不依赖 PP/MPC 包，不使用地图规划控制速度。

## 启动

```bash
ros2 launch maze_wall_pid merged.launch.py
```

唯一入口从 `(-2.45, 0.6701, 0.01)`、yaw=0 的内侧入口生成 Burger，巡航速度 1.5 m/s，默认关闭 SLAM。

加载 Gazebo、RViz 和雷达调试窗后，在启动终端输入 `start` 并回车。无窗口可添加 `gazebo_gui:=false use_rviz:=false debug_window:=false`。`mapping:=true` 仅开启地图显示，运动控制仍使用雷达、TF 和融合里程计。

## 文件

| 路径 | 用途 |
| --- | --- |
| `src/left2_lidar.cpp` | ROS 节点、连续墙点、PID、寻墙和出口停车 |
| `include/maze_wall_pid/left2_trace.hpp` | 墙线追踪、法向偏移、几何拟合、有界 PID |
| `config/merged_left2.yaml` | 合并图双墙几何辅助与 PID 参数 |
| `world/merged_maze.world` | 合并迷宫 |
| `rviz/merged_left2.rviz` | 合并图固定总览视角 |
| `test/test_left2_trace.cpp` | 墙线、PID 和双墙辅助回归 |

PID 参数为 4/0/12，巡航速度 1.5 m/s、出口开阔确认 0.10 s，保持原样。本轮删除地图与整理入口，没有重新调参。当前 1.5 m/s 尚未稳定通过全程验收。

识别出口后继续驶出，确认实测停稳再锁定 `EXIT_REACHED`；完成后持续发布零速度。

## 接口和参数

- `/wall_follow/status`、`/wall_follow/result`：JSON 状态及持久任务结果。
- `/wall_follow/set_enabled`（`std_srvs/srv/SetBool`）：暂停/恢复；完成后拒绝重新启用。
- `/cmd_vel`：速度指令，经 `velocity_guard.py` 限幅后送至 `/burger/cmd_vel`。
- `wall_params:=/absolute/path/config.yaml`：替换控制参数。
- `wait_for_start:=false`：显式自动开始。
- `debug_window:=false`：关闭独立雷达调试窗。

```bash
ros2 service call /wall_follow/set_enabled std_srvs/srv/SetBool '{data: true}'
ros2 service call /wall_follow/set_enabled std_srvs/srv/SetBool '{data: false}'
```

仿真准备代码位于 `launch_support/simulation.py`，仅由此入口内部加载；速度限幅 1.5 m/s，自定义速度时应同步调整 `guard_linear_speed`。独立构建使用 `colcon build --symlink-install --packages-up-to maze_wall_pid`。
