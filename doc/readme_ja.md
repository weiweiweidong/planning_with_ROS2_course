# Planning with ROS 2 Course

[简体中文](../readme.md) | [English](readme_en.md) | [日本語](readme_ja.md)

ROS 2 をベースにした自動運転の意思決定・計画アルゴリズム学習用プロジェクトです。PNC Map、グローバルパス、参照線、ローカルパス、速度計画、障害物との相互作用、軌跡合成、RViz 可視化を中心に、計画処理の一連の流れを実験できる構成になっています。

## 特徴

- ROS 2 ワークスペースとして構成され、C++ の計画ノード、Python の描画ノード、カスタムメッセージ/サービスを含みます。
- 直線道路や S 字道路などの PNC Map 生成に対応しています。
- 経路追従、静的障害物回避、同一車線上の障害物との相互作用、動的障害物回避などのシナリオを設定できます。
- グローバルパスサービス、ローカルパス計画、速度計画、参照線生成、平滑化モジュールを提供します。
- 自車/障害車の URDF モデル、TF 配信、RViz 設定、Matplotlib によるデータ描画を含みます。

## ディレクトリ構成

```text
.
├── doc/                         # コース資料
├── src/
│   ├── base_msgs/               # カスタム msg/srv インターフェース
│   ├── data_plot/               # Python 描画ノード
│   └── planning/                # 計画処理の中心パッケージ
│       ├── config/              # シナリオと計画パラメータ
│       ├── launch/              # launch ファイル
│       ├── rviz/                # RViz 設定
│       ├── urdf/                # 自車と障害車のモデル
│       └── src/                 # C++ 機能モジュール
├── frames_2026-01-09_02.32.54.* # エクスポートされた TF ツリーファイル
└── readme.md
```

## 機能モジュール

| パッケージ/モジュール | 説明 |
| --- | --- |
| `base_msgs` | `PNCMap`、`Referline`、`LocalPath`、`LocalSpeeds`、`LocalTrajectory`、`PlotInfo` などのメッセージと、`PNCMapService`、`GlobalPathService` サービスを定義します。 |
| `planning/src/pnc_map_creator` | PNC Map を生成し、サービス経由で計画フローに提供します。 |
| `planning/src/global_planner` | PNC Map に基づいてグローバルパスを生成します。 |
| `planning/src/reference_line` | ローカル参照線を生成し、平滑化します。 |
| `planning/src/decision_center` | 障害物と道路境界に基づいて意思決定の目標を生成します。 |
| `planning/src/local_planner` | ローカルパス、速度曲線、合成ローカル軌跡を生成します。 |
| `planning/src/move_cmd` | 軌跡に基づいて自車 TF を配信し、障害車 TF も配信します。 |
| `data_plot` | 計画用の描画データを購読し、Matplotlib でパス、速度、加速度などを可視化します。 |

## 環境要件

Ubuntu と ROS 2 デスクトップ環境での実行を推奨します。コードには ROS 2 Humble 関連の互換コメントが含まれているため、使用する ROS 2 ディストリビューションと依存関係のバージョンを環境に合わせてください。

基本依存関係:

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
- Python 依存関係: `numpy`、`matplotlib`

apt で ROS 2 依存関係をインストールする場合は、次の `<ros-distro>` を使用中の ROS 2 ディストリビューション名に置き換えてください。例: `humble`

```bash
source /opt/ros/<ros-distro>/setup.bash
```

一般的な実行時依存関係をインストールします:

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

> 注意: `Eigen3` と `OsqpEigen` のインストール方法は環境によって異なります。現在の `planning/CMakeLists.txt` では `/usr/local/include/eigen-3.4.1` を明示的に include しています。Eigen のインストールパスが異なる場合は、CMake 設定も合わせて変更してください。

## クイックスタート

### 1. リポジトリをクローン

```bash
git clone <your-repository-url>
cd planning_with_ROS2_course
```

### 2. ROS 2 環境を読み込む

```bash
source /opt/ros/<ros-distro>/setup.bash
```

pyenv を使用していて、このプロジェクトではシステム Python を使いたい場合は、プロジェクトルートの `.python-version` を保持してください。

```text
system
```

### 3. ビルド

```bash
colcon build
```

中心パッケージだけを再ビルドする場合:

```bash
colcon build --packages-select base_msgs planning data_plot
```

### 4. ワークスペース環境を読み込む

```bash
source install/setup.bash
```

新しいターミナルを開くたびに実行してください。必要に応じて shell 設定ファイルに追加してもかまいません。

### 5. 計画可視化を起動

```bash
ros2 launch planning planning_launch.py
```

この launch ファイルは次のノードを起動します:

- 自車と障害車の `robot_state_publisher`
- `joint_state_publisher`
- RViz2
- `data_plot`
- `pnc_map_server`
- `global_path_server`
- `planning_process`

### 6. 車両の移動指令を起動

別のターミナルを開きます:

```bash
source /opt/ros/<ros-distro>/setup.bash
source install/setup.bash
ros2 launch planning move_cmd_launch.py
```

この launch ファイルは次のノードを起動します:

- `car_move_cmd`: `/planning/local_trajectory` を購読し、自車 TF を配信します。
- `obs_move_cmd`: シナリオ設定に基づいて障害車 TF を配信します。

## シナリオ設定

シナリオの入口設定ファイル:

```text
src/planning/config/senario_config.yaml
```

対応している `scenario.type`:

| タイプ | シナリオ |
| --- | --- |
| `0` | 経路追従、障害物なし |
| `1` | 静的障害物回避 |
| `2` | 同一車線上の障害物との相互作用。停止、追従、追い越し、すれ違いを含む |
| `3` | 動的障害物回避 |

シナリオごとに異なる計画パラメータを読み込みます:

| シナリオタイプ | パラメータファイル |
| --- | --- |
| `0` / `1` | `planning_static_obs_config.yaml` |
| `2` | `planning_onlane_obs_config.yaml` |
| `3` | `planning_dynamic_obs_config.yaml` |

設定を変更した後は、`install/planning/share/planning/config` に反映されるように再ビルドまたは再インストールしてください:

```bash
colcon build --packages-select planning
source install/setup.bash
```

## 主な Topic と Service

`planning_launch.py` では、計画系の中心ノードが `/planning` 名前空間に配置されます。

| 名前 | 種別 | 説明 |
| --- | --- | --- |
| `/planning/pnc_map_server` | service | PNC Map を要求し、生成します。 |
| `/planning/global_path_server` | service | グローバルパスを要求し、生成します。 |
| `/planning/pnc_map` | topic | 計画用マップを publish します。 |
| `/planning/pnc_map_markerarray` | topic | RViz 用のマップ可視化 Marker を publish します。 |
| `/planning/global_path` | topic | グローバルパスを publish します。 |
| `/planning/global_path_rviz` | topic | RViz 用のグローバルパス Marker を publish します。 |
| `/planning/reference_line` | topic | 参照線を publish します。 |
| `/planning/local_path` | topic | ローカルパスを publish します。 |
| `/planning/local_trajectory` | topic | ローカル軌跡を publish します。 |
| `/planning/plot_info` | topic | 描画用データを publish します。 |

実行状態は次のコマンドで確認できます:

```bash
ros2 node list
ros2 topic list
ros2 service list
ros2 run tf2_tools view_frames
```

## 開発メモ

### パッケージを再ビルド

```bash
colcon build --packages-select planning
source install/setup.bash
```

### OSQP テストノードを実行

プロジェクトには OSQP のサンプルテストノードが含まれています:

```bash
ros2 run planning osqp_test
```

### 自車位置の更新方式を切り替える

`car_move_cmd` には `USE_ACTUAL_POS` コンパイルスイッチがあります。デフォルトでは無効で、軌跡のマッチング点を使って位置を更新します。有効にすると、積分した実位置を使います:

```bash
colcon build --packages-select planning --cmake-args -DUSE_ACTUAL_POS_MACRO=ON
source install/setup.bash
```

## FAQ

### `colcon build` 時に Python 環境エラーが出る

pyenv を使用している場合は、プロジェクトルートの `.python-version` が次の内容であることを確認してください:

```text
system
```

これによりプロジェクトはシステム Python を優先し、pyenv による ROS 2 Python パッケージパスの衝突を避けられます。

### Eigen または OsqpEigen が見つからない

Eigen3 と OsqpEigen がインストールされていることを確認し、`src/planning/CMakeLists.txt` 内の Eigen include パスがローカル環境と一致しているか確認してください。

### YAML を変更しても実行結果が変わらない

ROS 2 のインストール済みリソースは `install/` ディレクトリから読み込まれます。`src/planning/config/*.yaml` を変更した後は、再ビルドして workspace を source してください:

```bash
colcon build --packages-select planning
source install/setup.bash
```

## 参考資料

コース PDF は次の場所にあります:

```text
doc/【保姆级】基于ROS2的自动驾驶决策规划算法实战开发.pdf
```

## License

このプロジェクトでは現在、オープンソースライセンスが宣言されていません。GitHub で公開する予定がある場合は、`LICENSE` ファイルを追加し、各 `package.xml` の license フィールドも更新することをおすすめします。
