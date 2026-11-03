# 代码与地图来源

课程：`(2026-2027-1)-S0610350-02`，第11组，丁俊豪（24061435）、刘韩旭（24061412）。公开项目：<https://github.com/Zythyra/maze_ws>。

## 课程项目与改编范围

| 资源 | 原始来源 | 本项目使用与修改 |
| --- | --- | --- |
| Burger描述与仿真模型 | [turtlebot3 / humble](https://github.com/ROBOTIS-GIT/turtlebot3/tree/humble)、[turtlebot3_simulations / humble](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/humble) | `src/vendor/`保留上游资源；launch准备世界、生成模型并配置仿真接口 |
| PID巡线参考 | [ifly2_2026 / left2.cpp](https://github.com/Zythyra/ifly2_2026/blob/main/src/line_follow/src/left2.cpp) | 将视觉边界点替换为激光墙点，加入净宽估计、双墙融合、曲率/朝向控制及出口状态机 |
| 在线SLAM | [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) | 生成map定位、位姿图和占据栅格，辅助拓扑几何修正与已观测死路判定 |
| PP几何原理 | [Coulter, Implementation of the Pure Pursuit Path Tracking Algorithm](https://publications.ri.cmu.edu/implementation-of-the-pure-pursuit-path-tracking-algorithm) | 实现前瞻圆交点、位姿预测与差速追踪；与自建道路拓扑、Dijkstra及栅格A*返航集成 |
| 课程world | 提供的`maze2.world`、`merged_maze.world` | 配置入口出生位姿、统一场景入口及兼容光照；合并图保持原墙体几何 |

PP相关模块由丁俊豪负责，PID相关模块由刘韩旭负责。算法框架、方案取舍和实验分析由组员确定；AI辅助代码适配、排错与文档整理，具体范围见根目录README和课程报告。

## 上游固定版本与场景记录

官方 TurtleBot3 模型保留在 `src/vendor`，许可证与固定提交见各包的 `LICENSE` 和 `UPSTREAM.txt`。

PID 墙点提取与有界 PID 改编自 https://github.com/Zythyra/ifly2_2026 的 `left2.cpp`，blob `72924b249b756bd87030678057eca3e62a120ec0`；来源注释保留在头文件和 C++ 源码。PP/MPC 平滑参考与优化算法参考该仓库 `rk3588` 分支，提交 `4c876c9e551c4ad161344b9efa5a625f48a981e3`。

以下保留当前算法及合并地图的来源元数据。参考工程源码及原始地图副本保存在工作区外备份。

### ifly2_mpc_ros1

```json
{
  "repository": "https://github.com/Zythyra/ifly2_2026",
  "branch": "rk3588",
  "commit": "4c876c9e551c4ad161344b9efa5a625f48a981e3",
  "source_path": "src/my_planner",
  "purpose": "ROS1 algorithm reference; excluded from colcon build"
}
```

### merged_maze_20261027

```json
{
  "source": "/home/zythyra/merged_maze.world",
  "sha256": "38a03b61be1ad97445682d48e8d8d0490b881b579773594c20fb1c0f309ed885",
  "working_copy": "src/maze_wall_pid/world/merged_maze.world",
  "note": "Only unsupported ambient light converted to scene ambient; models and geometry preserved."
}
```
