# maze_bringup：PP 迷宫搜索与返航

课程`(2026-2027-1)-S0610350-02`，第11组。**模块负责人：丁俊豪，24061435。**

默认方案在maze2中同步维护道路拓扑和SLAM地图，左优先探索，通过Dijkstra转移到待探索节点，识别出口后以栅格A*规划返航，所有参考路线由PP跟踪。

## 构建与启动

```bash
cd ~/maze_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to maze_bringup
source install/local_setup.bash
ros2 launch maze_bringup maze2.launch.py
# 窗口就绪后在同一终端输入start并回车
```

默认从`(-2.275, -3.8, 0.01)`、yaw=π/2的下方入口朝+Y生成Burger。巡航速度上限0.53 m/s。入口集成world、机器人、Gazebo、RViz、SLAM和控制器。

## 拓扑图、SLAM与节点调度

- **拓扑图管理探索**：`Junction`保存四方向`ports`、道路ID及邻接表；`Leg`保存道路端点、连接和访问状态。左右缺口均登记；选择支路时标记`tried/visited`，取消尚未执行的预选支路则恢复。局部按左、前、右、后优先选择。
- **SLAM辅助几何**：当前扫描和里程计采用同一份`map→odom`修正；关键帧锚点在位姿图优化后更新历史节点、起点和参考线；墙面约束联合校正共享路口，每次修正最多0.04 m且落在已知空闲区域。
- **SLAM辅助死路判定**：`BranchMap.check`在实测入口墙间作虚拟封口，对原始自由栅格八邻域泛洪。与父走廊隔离、远离地图边界和未知区域的封闭口袋可提前剪枝；同时支持正前方确认死路后立即回溯。
- **Dijkstra调度目标节点**：`frontier_route`只扩展已建立邻接连接且标记`visited`的道路，边权为节点当前map坐标欧氏距离。优先队列中第一个仍有未探索支路的非起始节点即为最近探索目标，返回最短道路序列。
- **连续执行路线**：`start_return`保存`leg/return_route/next_leg`，`path`拼接道路端点供PP追踪，`advance`随进度切段。回溯沿用节点和道路ID；新发现的左支路可接管后续路线。

## PP与出口返航

`pursuit_target`取路径与前瞻圆的前向交点；`predict_pose`预测0.10 s位姿，再按`κ=2yL/L²`生成角速度，加入实际角速度反馈。探索前瞻0.32 m，返航前瞻0.28 m；大角度时先P控制原地对准，进入容差后前进。

经历走廊后，至少三侧开阔且满足连续开放角、0.8 s确认条件时识别出口。停车0.6 s后在已知安全栅格运行A*，包含0.20 m净空、靠墙代价及禁止对角穿墙。返航规划层可用当前实测激光补齐出口附近观测。参考线由PP跟踪，距地图起点小于0.10 m时锁定`HOME_REACHED`。

maze2记录的A*参考长16.839 m，出口至起点32.64 s，终止地图距离0.0951 m，详见[实验记录](../../docs/EXPERIMENTS.md)。

## 文件与分工

| 路径 | 用途 |
| --- | --- |
| `scripts/polyline_follower.py` | ROS接口、坐标修正、控制循环与返航阶段 |
| `scripts/polyline_core.py` | 激光几何、拓扑、Dijkstra、锚点及PP |
| `scripts/branch_map.py` | 已观测死路证明与剪枝 |
| `scripts/return_planner.py` | 栅格A*返航与补观测 |
| `config/polyline.yaml`、`config/slam.yaml` | 控制、探索、返航及SLAM参数 |
| `world/maze2.world`、`launch/maze2.launch.py` | 课程场景和一键入口 |
| `test/` | 拓扑、地图、返航和启动回归 |

上述探索、路线追踪与返航模块由丁俊豪负责，仿真接口由两位成员共同集成。

## 接口与参数

- `/polyline/status`、`/polyline/result`：任务状态及持久结果。
- `/polyline/global_path`、`/polyline/active_path`、`/polyline/trajectory`：参考路径和实际轨迹。
- `/polyline/astar_return_path`：返航路径。
- `/polyline/graph`、`/polyline/markers`：拓扑和可视化。
- `/polyline/set_enabled`（`std_srvs/srv/SetBool`）：暂停/恢复；完成后重新启动launch。

```bash
ros2 service call /polyline/set_enabled std_srvs/srv/SetBool '{data: false}'
ros2 service call /polyline/set_enabled std_srvs/srv/SetBool '{data: true}'
```

`gazebo_gui:=false use_rviz:=false`关闭窗口，`wait_for_start:=false`自动开始。`planner_params:=/absolute/path/config.yaml`、`slam_params:=...`替换参数；`world:=... x:=... y:=... yaw:=...`替换场景和出生位置。`mapping:=false`为odom调试模式，默认课程任务开启SLAM。

## 可选拓展

另保留`motion_reference.py`、`differential_mpc.py`及`mpc_maze2.launch.py`作为平滑参考与差速MPC实验。OSQP固定1.0.4，安装依赖后启动：

```bash
cd ~/maze_ws
python3 -m pip install --user --no-deps -r requirements-mpc.txt
ros2 launch maze_bringup mpc_maze2.launch.py
```

`controller_mode:=optimized_pp`为平滑参考PP对照模式；对应诊断话题为`/polyline/optimized_path`、`/polyline/mpc_reference_path`、`/polyline/mpc_predicted_path`。课程报告采用纯PP方案。外部参考及模型许可证见[来源记录](../../docs/SOURCES.md)。
