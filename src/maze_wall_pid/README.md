# maze_wall_pid：PID 左墙追踪

课程`(2026-2027-1)-S0610350-02`，第11组。**模块负责人：刘韩旭，24061412。**

本包在合并迷宫中执行激光墙线追踪、双墙几何辅助与出口停车。默认使用局部雷达、TF和融合里程计。

## 构建与启动

```bash
cd ~/maze_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to maze_wall_pid
source install/local_setup.bash
ros2 launch maze_wall_pid merged.launch.py
# 窗口就绪后在同一终端输入start并回车
```

默认出生位置`(-2.45, 0.6701, 0.01)`，yaw=0，从合并迷宫内侧入口沿+X进入。默认巡航速度1.5 m/s，SLAM关闭。`mapping:=true`可开启建图显示，控制器仍使用局部墙线闭环。

## 算法

1. `trace_edge`按角度跟踪连续墙点，遇真实缺口终止；平滑后以0.015 m弧长重采样，法向偏移得到参考线。
2. `update_corridor`由左右墙面的垂直距离估计净宽；可靠双墙近直线观测通过残差与方向检查后，以0.60 s滤波和0.40 m/s变化率更新。目标墙距取净宽一半，初值0.85 m。
3. `wall_geometry`拟合墙面参数曲线，计算横向误差、切向朝向与偏移曲率；右墙镜像复用同一计算。`corridor_geometry`按可见弧长与拟合残差融合双墙，冲突时选较可靠一侧。
4. 正常控制为`ωdes = vκ + 3veψ + PID(ey)`，几何失效时使用近端加权横向误差。PID为4/0/12，D项是限幅帧间差分；随后加入0.30倍偏航响应误差，限制角速度±2.0 rad/s、变化率4.0 rad/s²。
5. 状态优先级：完成/等待/数据有效性 → 出口处理 → 前险右转 → 连续丢线左寻墙 → 正常追踪 → 短缺线保持。前方小于0.50 m时原地右转；连续2帧丢线时向左寻墙。
6. 识别出口后，开阔持续0.10 s，保持航向驶出至少0.30 m，进入制动。实测停稳持续0.50 s后锁定`EXIT_REACHED`并持续零速度。

最终1.5 m/s归档轮次约58.08 s正常完成，轨迹、速度与三轮对比见[实验记录](../../docs/EXPERIMENTS.md)。

## 文件与分工

| 路径 | 用途 |
| --- | --- |
| `src/left2_lidar.cpp` | ROS节点、扫描、净宽估计、状态机、控制与停车 |
| `include/maze_wall_pid/left2_trace.hpp` | 墙线、几何拟合、双墙融合及PID |
| `config/merged_left2.yaml` | 合并迷宫控制参数 |
| `world/merged_maze.world` | 合并迷宫 |
| `launch/merged.launch.py` | 一键加载场景和节点 |
| `rviz/merged_left2.rviz` | 固定视角、雷达和控制调试显示 |
| `test/test_left2_trace.cpp` | 墙线、PID及双墙辅助回归 |

上述控制算法、参数与实验分析由刘韩旭负责，仿真接口由两位成员共同集成。

## 接口与参数

- `/wall_follow/status`、`/wall_follow/result`：JSON状态和持久任务结果。
- `/wall_follow/set_enabled`（`std_srvs/srv/SetBool`）：暂停/恢复；完成后重新启动launch进行下一轮。
- `/cmd_vel`：经`velocity_guard.py`转发至`/burger/cmd_vel`。
- `wall_params:=/absolute/path/config.yaml`：替换控制参数。
- `gazebo_gui:=false use_rviz:=false debug_window:=false`：无窗口运行。
- `wait_for_start:=false`：自动开始。

```bash
ros2 service call /wall_follow/set_enabled std_srvs/srv/SetBool '{data: false}'
ros2 service call /wall_follow/set_enabled std_srvs/srv/SetBool '{data: true}'
```

仿真准备模块为`launch_support/simulation.py`。速度保护默认线速度上限1.5 m/s，对应参数`guard_linear_speed`。模型、代码来源和许可证见[来源记录](../../docs/SOURCES.md)。
