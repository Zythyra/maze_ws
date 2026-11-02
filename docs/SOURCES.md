# 代码与地图来源

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
