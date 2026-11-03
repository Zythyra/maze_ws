# ROS 2 迷宫机器人仿真

使用 TurtleBot3 Burger，在 Gazebo 中运行 PID 左墙追踪与 PP 迷宫搜索。

## 项目环境

| 组件 | 版本或依赖 |
| --- | --- |
| 操作系统 | Ubuntu 22.04 LTS |
| ROS 2 | Humble |
| 仿真器 | Gazebo Classic 11 |
| 机器人 | TurtleBot3 Burger |
| 可视化 | RViz 2 |
| 在线建图 | slam_toolbox |
| 开发依赖 | Python 3、C++、NumPy、SciPy、OpenCV |

`src/vendor/`已包含机器人描述和仿真模型。以下步骤在已安装 ROS 2 Humble 的环境中执行。

## 编译

安装构建工具：

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep
```

下载源码、安装依赖并编译：

```bash
git clone https://github.com/Zythyra/maze_ws.git ~/maze_ws
cd ~/maze_ws
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
MAKEFLAGS=-j2 CMAKE_BUILD_PARALLEL_LEVEL=2 \
  colcon build --symlink-install --executor sequential \
  --packages-up-to maze_wall_pid maze_bringup
source install/local_setup.bash
```

首次使用 rosdep 时，在 `rosdep update` 前执行一次 `sudo rosdep init`。若使用解压后的工作空间，直接进入 `~/maze_ws`，跳过克隆步骤。

## 启动

每个新终端先加载环境：

```bash
source /opt/ros/humble/setup.bash
source ~/maze_ws/install/local_setup.bash
```

选择以下一个入口运行，launch 会同时启动 Gazebo、RViz 和控制节点。

**PID 左墙追踪**：加载 `merged_maze.world`。

```bash
ros2 launch maze_wall_pid merged.launch.py
```

**PP 迷宫搜索与返航**：加载 `maze2.world`，同时启动在线建图。

```bash
ros2 launch maze_bringup maze2.launch.py
```

窗口加载完成后，在启动 launch 的同一终端输入 `start` 并回车，小车开始运行。按 `Ctrl+C` 结束本次仿真；重新运行时再次启动对应 launch。
