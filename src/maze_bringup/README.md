# maze_bringup：PP / MPC 最终代码

该包仅保留两个启动入口，分别运行 maze2 的 PP 和 MPC 方案。仿真准备代码位于 `launch_support/simulation.py`，作为入口内部模块加载，不提供独立 launch。

## 启动

```bash
ros2 launch maze_bringup maze2.launch.py       # PP，0.53 m/s
ros2 launch maze_bringup mpc_maze2.launch.py   # MPC，上限 0.60 m/s
```

等待窗口加载，在同一个终端输入 `start` 并回车。无交互终端可主动调用 `/polyline/set_enabled`；`wait_for_start:=false` 允许自动开始。两个控制版本共用拓扑探索、SLAM 修正、死路跳过/回溯、出口识别和 A* 返航，最终状态为 `HOME_REACHED`。`controller_mode:=optimized_pp` 保留平滑参考的 PP 对照模式。

## 文件

| 路径 | 用途 |
| --- | --- |
| `scripts/polyline_follower.py` | ROS 节点、状态机、PP/MPC 选择、返航 |
| `scripts/polyline_core.py` | 雷达几何、道路拓扑、PP |
| `scripts/branch_map.py` | 地图约束和死支路判定 |
| `scripts/return_planner.py` | 占据栅格 A* 返航 |
| `scripts/motion_reference.py` | 已观测走廊中的平滑参考和速度曲线 |
| `scripts/differential_mpc.py` | OSQP 差速 MPC、轮速约束、回退 |
| `config/polyline.yaml` | PP、规划、返航与 MPC 参数 |
| `config/slam.yaml` | SLAM 参数 |
| `world/` | maze2 |
| `test/` | 最终算法回归和只读诊断 |

MPC 使用 OSQP 1.0.4；固定 Python 依赖见工作空间根目录 `requirements-mpc.txt`。NumPy、SciPy、OpenCV 使用 Ubuntu/ROS 的系统包。

## 接口

- `/polyline/status`、`/polyline/result`：JSON 状态和任务结果。
- `/polyline/global_path`、`/polyline/active_path`、`/polyline/trajectory`：路径及轨迹。
- `/polyline/optimized_path`、`/polyline/mpc_reference_path`、`/polyline/mpc_predicted_path`：平滑/MPC 诊断。
- `/polyline/astar_return_path`：返航路径。
- `/polyline/graph`、`/polyline/markers`：探索拓扑。
- `/polyline/set_enabled`（`std_srvs/srv/SetBool`）：暂停/恢复；完成后拒绝重启任务。

```bash
ros2 service call /polyline/set_enabled std_srvs/srv/SetBool '{data: true}'
ros2 service call /polyline/set_enabled std_srvs/srv/SetBool '{data: false}'
```

`mapping:=false` 使用 odom 调试模式；默认开启地图。可用 `planner_params:=/absolute/path/config.yaml`、`slam_params:=...` 替换配置，用 `world:=... x:=... y:=... yaw:=...` 覆盖场景及出生点。
