# Planning with ROS 2 Course

[简体中文](../readme.md) | [English](readme_en.md) | [日本語](readme_ja.md)

A ROS 2 based autonomous driving decision-making and planning course project. The project is designed for learning and experimentation, and it builds a complete planning workflow around PNC Map generation, global path planning, reference line generation, local path planning, speed planning, obstacle interaction, trajectory composition, and RViz visualization.

## Features

- Organized as a ROS 2 workspace, including C++ planning nodes, a Python plotting node, and custom messages/services.
- Supports PNC Map generation for straight roads and S-turn roads.
- Supports route following, static obstacle avoidance, same-lane obstacle interaction, and dynamic obstacle avoidance scenarios.
- Provides global path services, local path planning, speed planning, reference line generation, and smoothing modules.
- Provides ego vehicle/obstacle vehicle URDF models, TF broadcasting, RViz configuration, and Matplotlib data plotting.

## Project Structure

```text
.
├── doc/                         # Course documents
├── src/
│   ├── base_msgs/               # Custom msg/srv interfaces
│   ├── data_plot/               # Python plotting node
│   └── planning/                # Core planning package
│       ├── config/              # Scenario and planning parameters
│       ├── launch/              # Launch files
│       ├── rviz/                # RViz configuration
│       ├── urdf/                # Ego and obstacle vehicle models
│       └── src/                 # C++ functional modules
├── frames_2026-01-09_02.32.54.* # Exported TF tree files
└── readme.md
```

## Modules

| Package/Module | Description |
| --- | --- |
| `base_msgs` | Defines messages such as `PNCMap`, `Referline`, `LocalPath`, `LocalSpeeds`, `LocalTrajectory`, and `PlotInfo`, plus the `PNCMapService` and `GlobalPathService` services. |
| `planning/src/pnc_map_creator` | Generates the PNC Map and provides it to the planning workflow through a service. |
| `planning/src/global_planner` | Generates a global path based on the PNC Map. |
| `planning/src/reference_line` | Generates and smooths the local reference line. |
| `planning/src/decision_center` | Generates decision targets based on obstacles and road boundaries. |
| `planning/src/local_planner` | Generates the local path, speed curve, and composed local trajectory. |
| `planning/src/move_cmd` | Broadcasts the ego vehicle TF based on the trajectory and broadcasts obstacle vehicle TFs. |
| `data_plot` | Subscribes to planning plot data and visualizes path, speed, acceleration, and related information with Matplotlib. |

## Requirements

Ubuntu with a ROS 2 desktop environment is recommended. The code includes compatibility comments related to ROS 2 Humble, so make sure your local ROS 2 distribution and dependency versions are consistent with your environment.

Basic dependencies:

- ROS 2
- `colcon`
- `ament_cmake`
- `ament_python`
- `rclcpp`, `rclpy`
- `tf2`, `tf2_ros`
- `geometry_msgs`, `nav_msgs`, `visualization_msgs`, `std_msgs`
- `robot_state_publisher`
- `joint_state_publisher`
- `rviz2`
- `xacro`
- `yaml-cpp`
- `Eigen3`
- `OsqpEigen`
- Python dependencies: `numpy`, `matplotlib`

If you install ROS 2 dependencies with apt, replace `<ros-distro>` with your ROS 2 distribution name, for example `humble`:

```bash
source /opt/ros/<ros-distro>/setup.bash
```

Install common runtime dependencies:

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

> Note: Installation methods for `Eigen3` and `OsqpEigen` may vary by system. The current `planning/CMakeLists.txt` explicitly includes `/usr/local/include/eigen-3.4.1`. If your Eigen installation path is different, update the CMake configuration accordingly.

## Quick Start

### 1. Clone the Repository

```bash
git clone <your-repository-url>
cd planning_with_ROS2_course
```

### 2. Source ROS 2

```bash
source /opt/ros/<ros-distro>/setup.bash
```

If your machine uses pyenv and this project should use the system Python, keep the `.python-version` file in the project root:

```text
system
```

### 3. Build

```bash
colcon build
```

To rebuild only the core packages:

```bash
colcon build --packages-select base_msgs planning data_plot
```

### 4. Source the Workspace

```bash
source install/setup.bash
```

Run this command whenever you open a new terminal. You may also add it to your shell configuration if needed.

### 5. Launch Planning Visualization

```bash
ros2 launch planning planning_launch.py
```

This launch file starts:

- `robot_state_publisher` for the ego vehicle and obstacle vehicle
- `joint_state_publisher`
- RViz2
- `data_plot`
- `pnc_map_server`
- `global_path_server`
- `planning_process`

### 6. Launch Vehicle Motion Commands

Open another terminal:

```bash
source /opt/ros/<ros-distro>/setup.bash
source install/setup.bash
ros2 launch planning move_cmd_launch.py
```

This launch file starts:

- `car_move_cmd`: subscribes to `/planning/local_trajectory` and broadcasts the ego vehicle TF.
- `obs_move_cmd`: broadcasts obstacle vehicle TFs according to the scenario configuration.

## Scenario Configuration

Scenario entry configuration file:

```text
src/planning/config/senario_config.yaml
```

Supported `scenario.type` values:

| Type | Scenario |
| --- | --- |
| `0` | Route following, no obstacles |
| `1` | Static obstacle avoidance |
| `2` | Same-lane obstacle interaction, including stopping, following, overtaking, and meeting |
| `3` | Dynamic obstacle avoidance |

Different scenarios load different planning parameter files:

| Scenario Type | Parameter File |
| --- | --- |
| `0` / `1` | `planning_static_obs_config.yaml` |
| `2` | `planning_onlane_obs_config.yaml` |
| `3` | `planning_dynamic_obs_config.yaml` |

After modifying configuration files, rebuild or reinstall resources so that `install/planning/share/planning/config` is updated:

```bash
colcon build --packages-select planning
source install/setup.bash
```

## Main Topics and Services

In `planning_launch.py`, core planning nodes are placed under the `/planning` namespace.

| Name | Type | Description |
| --- | --- | --- |
| `/planning/pnc_map_server` | service | Requests and generates the PNC Map. |
| `/planning/global_path_server` | service | Requests and generates the global path. |
| `/planning/pnc_map` | topic | Publishes the planning map. |
| `/planning/pnc_map_markerarray` | topic | Publishes RViz map visualization markers. |
| `/planning/global_path` | topic | Publishes the global path. |
| `/planning/global_path_rviz` | topic | Publishes the RViz global path marker. |
| `/planning/reference_line` | topic | Publishes the reference line. |
| `/planning/local_path` | topic | Publishes the local path. |
| `/planning/local_trajectory` | topic | Publishes the local trajectory. |
| `/planning/plot_info` | topic | Publishes plotting data. |

Use the following commands to inspect runtime status:

```bash
ros2 node list
ros2 topic list
ros2 service list
ros2 run tf2_tools view_frames
```

## Development Notes

### Rebuild a Package

```bash
colcon build --packages-select planning
source install/setup.bash
```

### Run the OSQP Test Node

The project includes an OSQP example test node:

```bash
ros2 run planning osqp_test
```

### Switch Ego Vehicle Position Update Mode

`car_move_cmd` provides a `USE_ACTUAL_POS` compile-time switch. It is disabled by default and uses the trajectory matching point for position updates. When enabled, it uses the integrated actual position:

```bash
colcon build --packages-select planning --cmake-args -DUSE_ACTUAL_POS_MACRO=ON
source install/setup.bash
```

## FAQ

### Python Environment Error During `colcon build`

If your machine uses pyenv, confirm that the project root `.python-version` contains:

```text
system
```

This lets the project prefer the system Python and avoids ROS 2 Python package path conflicts caused by pyenv.

### Eigen or OsqpEigen Not Found

Make sure Eigen3 and OsqpEigen are installed, and check whether the Eigen include path in `src/planning/CMakeLists.txt` matches your local machine.

### YAML Changes Do Not Affect Runtime Behavior

ROS 2 installed resources are loaded from the `install/` directory. After modifying `src/planning/config/*.yaml`, rebuild and source the workspace:

```bash
colcon build --packages-select planning
source install/setup.bash
```

## Reference Document

The course PDF is located at:

```text
doc/【保姆级】基于ROS2的自动驾驶决策规划算法实战开发.pdf
```

## License

This project does not currently declare an open-source license. If you plan to publish it on GitHub, consider adding a `LICENSE` file and updating the license fields in each `package.xml`.
