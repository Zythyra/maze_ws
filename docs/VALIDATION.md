# 当前工作空间验证（2026-11-02）

项目地图为 maze2（PP/MPC）和合并迷宫（PID）。对外仅保留三个 launch 入口：maze2.launch.py、mpc_maze2.launch.py、merged.launch.py。每个入口集成地图、Gazebo、RViz 及对应控制器。

## 构建与回归

清空两个项目包的 build/install 后重新构建成功，安装目录没有已删除的入口和预览资源残留。

`colcon test --packages-select maze_bringup maze_wall_pid` 通过：157 tests，0 errors，0 failures，0 skipped；包含 131 项 Python 用例、18 项 C++ 用例及 8 项套件汇总。

三个 launch 文件均通过导入及 `ros2 launch <包> <入口> --show-args`。安装后的启动文件列表与源码一致。内部 `launch_support/simulation.py` 模块仅提供 Gazebo 准备函数，不含独立启动入口；地图、模型、速度保护、融合里程计、控制器和 RViz 仍由三个正式入口统一启动。

算法脚本、C++ 控制器、头文件、算法测试、控制参数及两张地图与修改前比较均未改变。移除了预览专用 RViz 配置和 joint_state_publisher / joint_state_publisher_gui 包依赖；仿真关节状态仍由原 Gazebo 插件发布。

## 仿真启动检查

在隔离的 ROS Domain / Gazebo master 中依次运行以下三个入口，关闭 GUI。每次实际加载地图及 Burger，检查 360 点激光、融合里程计、WAIT_START 持续零速度、服务启用后的非零速度指令，以及服务暂停后的持续零速度；随后关闭该轮进程。

| 包 | 入口 | 结果 |
| --- | --- | --- |
| `maze_bringup` | `maze2.launch.py` | 通过 |
| `maze_bringup` | `mpc_maze2.launch.py` | 通过 |
| `maze_wall_pid` | `merged.launch.py` | 通过 |

检查数据见 `launch-checks.json` 和 `runtime-checks.json`。这些检查验证资源定位、依赖、节点启动和控制门控；本轮未重跑完整探索/返航或合并图全程，也未进行实车测试或 GUI 交互测试。

合并图配置为`linear_speed=1.5`、`exit_open_confirm_time=0.10`。2026年11月3日整理课程材料时，从10月27日的原始记录恢复了最终完整轮次：约58.08 s正常驶出并停稳，保持`EXIT_REACHED`。该轮21帧制动状态和后续停车结果见[实验记录](EXPERIMENTS.md)及[原始结果摘要](results/pid_runs.json)。11月2日的启动检查与10月完整路线实验分别记录。

## 课程两个项目的测试与实验资料

课程`(2026-2027-1)-S0610350-02`、第11组报告采用PID左墙追踪和纯PP迷宫搜索。与这两个项目直接相关的五组测试共125个用例：墙线/PID 18、折线拓扑63、返航13、地图死路24、启动门控7，全部通过。上文157项为包含拓展控制器及套件汇总的完整工作空间统计。

PP的10月20日maze2记录完成`HOME_REACHED`，A*参考长度16.839 m、出口至起点32.64 s（仿真时间），终止地图距离0.0951 m。完整实验条件、轮次与可视化见[EXPERIMENTS](EXPERIMENTS.md)。

## 本机备份

修改前源码快照位于工作空间外 `~/maze_ws_backups/maze_ws_before_launch_cleanup_20261102_222234.tar.gz`，对应 Git 历史保存在同目录的 bundle。备份不上传 GitHub。build/install/log 为验证生成，由 .gitignore 排除。
