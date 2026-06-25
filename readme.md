# Planning with ROS 2 Course

[简体中文](readme.md) | [English](doc/readme_en.md) | [日本語](doc/readme_ja.md)

一个基于 ROS 2 的自动驾驶决策规划课程项目。项目以教学和实验为目标，围绕 PNC Map、全局路径、参考线、局部路径、速度规划、障碍物交互、轨迹合成与 RViz 可视化搭建了一套完整的规划流程。

## 项目特性

- 基于 ROS 2 工作区组织，包含 C++ 规划节点、Python 绘图节点和自定义消息/服务。
- 支持直线路段、S 弯路段等 PNC Map 场景生成。
- 支持循迹、静态绕障、同车道障碍物交互、动态避障等规划场景配置。
- 提供全局路径服务、局部路径规划、速度规划、参考线生成和平滑模块。
- 提供主车/障碍车 URDF 模型、TF 广播、RViz 配置和 Matplotlib 数据绘图。

## 目录结构

```text
.
├── doc/                         # 课程文档
├── src/
│   ├── base_msgs/               # 自定义 msg/srv 接口
│   ├── data_plot/               # Python 绘图节点
│   └── planning/                # 规划核心包
│       ├── config/              # 场景和规划参数
│       ├── launch/              # 启动文件
│       ├── rviz/                # RViz 配置
│       ├── urdf/                # 主车和障碍车模型
│       └── src/                 # C++ 功能模块
├── frames_2026-01-09_02.32.54.* # TF 树导出文件
└── readme.md
```

## 功能模块

| 包/模块                        | 说明                                                                                                                                                         |
| ------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `base_msgs`                    | 定义规划系统使用的 `PNCMap`、`Referline`、`LocalPath`、`LocalSpeeds`、`LocalTrajectory`、`PlotInfo` 等消息，以及 `PNCMapService`、`GlobalPathService` 服务。 |
| `planning/src/pnc_map_creator` | 生成 PNC Map，并通过服务向规划流程提供地图。                                                                                                                 |
| `planning/src/global_planner`  | 基于 PNC Map 生成全局路径。                                                                                                                                  |
| `planning/src/reference_line`  | 生成并平滑局部参考线。                                                                                                                                       |
| `planning/src/decision_center` | 根据障碍物和道路边界生成决策目标。                                                                                                                           |
| `planning/src/local_planner`   | 生成局部路径、速度曲线，并合成局部轨迹。                                                                                                                     |
| `planning/src/move_cmd`        | 根据轨迹广播主车 TF，并广播障碍物 TF。                                                                                                                       |
| `data_plot`                    | 订阅规划绘图数据并用 Matplotlib 展示路径、速度、加速度等信息。                                                                                               |

## 环境要求

建议使用 Ubuntu + ROS 2 桌面环境运行。本项目代码中包含 ROS 2 Humble 相关兼容注释，实际使用时请确保本机 ROS 2 发行版与依赖版本一致。

基础依赖：

- ROS 2
- `colcon`
- `ament_cmake`
- `ament_python`
- `rclcpp`、`rclpy`
- `tf2`、`tf2_ros`
- `geometry_msgs`、`nav_msgs`、`visualization_msgs`、`std_msgs`
- `robot_state_publisher`
- `joint_state_publisher`
- `rviz2`
- `xacro`
- `yaml-cpp`
- `Eigen3`
- `OsqpEigen`
- Python 依赖：`numpy`、`matplotlib`

如果你使用 apt 安装 ROS 2 依赖，请将下面命令中的 `<ros-distro>` 替换为你的 ROS 2 发行版名称，例如 `humble`：

```bash
source /opt/ros/<ros-distro>/setup.bash
```

然后安装常见运行依赖：

```bash
sudo apt update
sudo apt install \
  ros-<ros-distro>-robot-state-publisher \
  ros-<ros-distro>-joint-state-publisher \
  ros-<ros-distro>-rviz2 \
  ros-<ros-distro>-xacro \
  ros-<ros-distro>-tf2-tools \
  libyaml-cpp-dev \
  python3-numpy \
  python3-matplotlib
```

> 注意：`Eigen3` 和 `OsqpEigen` 的安装方式可能随系统环境不同而变化。当前 `planning/CMakeLists.txt` 中显式包含了 `/usr/local/include/eigen-3.4.1`，如果你的 Eigen 安装路径不同，需要同步调整 CMake 配置。

## 快速开始

### 1. 克隆项目

```bash
git clone <your-repository-url>
cd planning_with_ROS2_course
```

### 2. 设置 ROS 2 环境

```bash
source /opt/ros/<ros-distro>/setup.bash
```

如果本机安装了 pyenv，而本项目希望使用系统 Python，请保留项目根目录中的 `.python-version` 文件：

```text
system
```

### 3. 编译

```bash
colcon build
```

如果只想重新编译核心包：

```bash
colcon build --packages-select base_msgs planning data_plot
```

### 4. 加载工作区环境

```bash
source install/setup.bash
```

建议每次打开新终端后都执行一次。也可以按需加入你的 shell 配置文件。

### 5. 启动规划可视化

```bash
ros2 launch planning planning_launch.py
```

该启动文件会启动：

- 主车和障碍车的 `robot_state_publisher`
- `joint_state_publisher`
- RViz2
- `data_plot`
- `pnc_map_server`
- `global_path_server`
- `planning_process`

### 6. 启动车辆运动指令

另开一个终端：

```bash
source /opt/ros/<ros-distro>/setup.bash
source install/setup.bash
ros2 launch planning move_cmd_launch.py
```

该启动文件会启动：

- `car_move_cmd`：订阅 `/planning/local_trajectory` 并广播主车 TF。
- `obs_move_cmd`：根据场景配置广播障碍物 TF。

## 场景配置

场景入口配置文件：

```text
src/planning/config/senario_config.yaml
```

当前支持的 `scenario.type`：

| 类型 | 场景                                         |
| ---- | -------------------------------------------- |
| `0`  | 循迹，无障碍物                               |
| `1`  | 静态绕障                                     |
| `2`  | 同车道障碍物交互，包括停障、跟车、超车、会车 |
| `3`  | 动态避障                                     |

不同场景会读取不同规划参数：

| 场景类型  | 参数文件                           |
| --------- | ---------------------------------- |
| `0` / `1` | `planning_static_obs_config.yaml`  |
| `2`       | `planning_onlane_obs_config.yaml`  |
| `3`       | `planning_dynamic_obs_config.yaml` |

修改配置后需要重新编译或重新安装资源文件，确保 `install/planning/share/planning/config` 中的配置同步更新：

```bash
colcon build --packages-select planning
source install/setup.bash
```

## 主要话题和服务

在 `planning_launch.py` 中，规划核心节点被放在 `/planning` 命名空间下。

| 名称                            | 类型    | 说明                          |
| ------------------------------- | ------- | ----------------------------- |
| `/planning/pnc_map_server`      | service | 请求并生成 PNC Map。          |
| `/planning/global_path_server`  | service | 请求并生成全局路径。          |
| `/planning/pnc_map`             | topic   | 发布规划地图。                |
| `/planning/pnc_map_markerarray` | topic   | 发布 RViz 地图可视化 Marker。 |
| `/planning/global_path`         | topic   | 发布全局路径。                |
| `/planning/global_path_rviz`    | topic   | 发布 RViz 全局路径 Marker。   |
| `/planning/reference_line`      | topic   | 发布参考线。                  |
| `/planning/local_path`          | topic   | 发布局部路径。                |
| `/planning/local_trajectory`    | topic   | 发布局部轨迹。                |
| `/planning/plot_info`           | topic   | 发布绘图数据。                |

可以用下面的命令检查运行状态：

```bash
ros2 node list
ros2 topic list
ros2 service list
ros2 run tf2_tools view_frames
```

## 开发说明

### 重新编译某个包

```bash
colcon build --packages-select planning
source install/setup.bash
```

### 运行 OSQP 测试节点

项目中提供了一个 OSQP 示例测试节点：

```bash
ros2 run planning osqp_test
```

### 切换主车位置更新方式

`car_move_cmd` 中提供了 `USE_ACTUAL_POS` 编译开关。默认关闭，使用轨迹匹配点位置更新；开启后使用实际积分位置更新：

```bash
colcon build --packages-select planning --cmake-args -DUSE_ACTUAL_POS_MACRO=ON
source install/setup.bash
```

## 常见问题

### `colcon build` 时 Python 环境异常

如果本机使用 pyenv，请确认项目根目录 `.python-version` 内容为：

```text
system
```

这样当前项目会优先使用系统 Python，避免 ROS 2 Python 包路径被 pyenv 环境覆盖。

### 找不到 Eigen 或 OsqpEigen

确认本机已安装 Eigen3 和 OsqpEigen，并检查 `src/planning/CMakeLists.txt` 中的 Eigen 头文件路径是否与你本机一致。

### 修改 YAML 后运行结果没有变化

ROS 2 安装资源来自 `install/` 目录。修改 `src/planning/config/*.yaml` 后，请重新编译并重新 source：

```bash
colcon build --packages-select planning
source install/setup.bash
```

## 参考文档

课程 PDF 位于：

```text
doc/【保姆级】基于ROS2的自动驾驶决策规划算法实战开发.pdf
```

## License

当前项目尚未声明开源许可证。若计划公开发布到 GitHub，建议补充 `LICENSE` 文件，并在各 `package.xml` 中同步更新 license 字段。
