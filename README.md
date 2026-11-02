# TurtleBot3 Burger 迷宫项目

Ubuntu 22.04、ROS 2 Humble、Gazebo Classic 11。工作空间只保留 maze2 和合并迷宫，按控制方案分为两个项目包：

| 包 | 内容 | 主要入口 |
| --- | --- | --- |
| `maze_bringup` | PP / 差速 MPC、SLAM、拓扑探索、死路回溯、出口识别、A* 返航 | `maze2.launch.py`、`mpc_maze2.launch.py` |
| `maze_wall_pid` | 合并迷宫的雷达墙点拟合、有界 PID、双墙几何辅助、出口停车 | `merged.launch.py` |

两包可以分别构建、运行，不依赖对方。`src/vendor/` 保留它们共同需要的两个官方 TurtleBot3 依赖包、模型和许可证。两项目包各带仿真入口、里程计融合、速度限幅及终端启动模块；共享模块使用相同副本，避免 PID 包依赖 PP/MPC 算法。

## 构建

```bash
cd ~/maze_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
# OSQP 的固定版本；保留系统 NumPy/SciPy，避免 pip 替换 ROS 使用的数值库。
python3 -m pip install --user --no-deps -r requirements-mpc.txt
MAKEFLAGS=-j2 CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --symlink-install --executor sequential
source install/local_setup.bash
```

本机已具备所需依赖。新终端使用前加载 `/opt/ros/humble/setup.bash` 和本工作空间的 `install/local_setup.bash`。仅构建单个方案可使用 `colcon build --symlink-install --packages-up-to maze_bringup` 或 `--packages-up-to maze_wall_pid`。

## PP / MPC

```bash
# PP，maze2，默认 0.53 m/s
ros2 launch maze_bringup maze2.launch.py

# 差速 MPC，maze2，速度上限 0.60 m/s
ros2 launch maze_bringup mpc_maze2.launch.py
```

每次选择一个入口运行。窗口准备后，在启动 launch 的同一个终端输入 `start` 并回车。默认等待启动，随后探索、识别出口、规划返航，回到起点并停稳后报告 `HOME_REACHED`。不读取 world 真值控制车辆。更多参数、接口和代码说明见 [PP/MPC 包](src/maze_bringup/README.md)。

## PID 寻左墙

```bash
# 合并迷宫：双墙几何辅助 + PID，当前配置 1.5 m/s
ros2 launch maze_wall_pid merged.launch.py
```

同样在启动终端输入 `start` 后开始。合并图检测出口、驶出并确认停稳后锁定 `EXIT_REACHED`，不执行 PP/MPC 的 A* 返航。PID 算法与合并图参数保持原样；当前 1.5 m/s 属于提速试验配置，尚未稳定通过全程验收。更多说明见 [PID 包](src/maze_wall_pid/README.md)。

## 仿真与操作

自动入口默认开启 Gazebo 和 RViz。无窗口检查可以加 `gazebo_gui:=false use_rviz:=false`；PID 还可加 `debug_window:=false`。`wait_for_start:=false` 显式自动开始。

整个项目仅保留上述三个 launch 入口，每个都集成相应地图、Gazebo、RViz 和控制器。两个方案使用 `/cmd_vel`、`/scan`、`/odom` 等相同话题，每次运行一个入口。键盘控制工具仍可用 `ros2 run <包名> teleop_keyboard.py` 调用。

## 检查

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
colcon test --packages-select maze_bringup maze_wall_pid --event-handlers console_direct+
colcon test-result --verbose
```

测试保留最终算法的回归用例，包括 PP、MPC、参考曲线、岔路地图、返航、启动门控和 C++ 墙线/PID。仿真启动后的只读传感器检查：

```bash
python3 src/maze_bringup/test/smoke_test.py
# PID 场景也可用 src/maze_wall_pid/test/smoke_test.py
```

本次整理的实际验证范围见 [验证记录](docs/VALIDATION.md)。代码与地图来源见 [来源记录](docs/SOURCES.md)，官方依赖保留原许可证。

## 仓库范围

仓库提交整个工作空间的最终源码、地图、配置、测试和使用说明。`build/`、`install/`、`log/`、Python 缓存不提交，克隆后重新构建。业务地图仅有 `src/maze_bringup/world/maze2.world` 与 `src/maze_wall_pid/world/merged_maze.world`。`test/worlds/` 保留算法回归使用的小场景，`src/vendor/` 保留官方依赖的原始资源。备份保存在本机工作空间之外，不上传 GitHub。
