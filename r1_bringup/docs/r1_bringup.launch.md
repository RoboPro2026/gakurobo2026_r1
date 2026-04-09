# r1_bringup.launch.py

通常の R1 起動に使うメイン launch です。実機モードとシミュレーションモードを `use_sim` で切り替え、`r1_main_node` の起動モードを `robot_control_mode` で切り替えます。parameter file で定義した `/cmd_vel_target -> r1_chassis_velocity_control_node -> /cmd_vel` の速度補正経路もここで組み立てます。必要に応じて `r1_ui` の ArUco 表示ノードも起動できます。

## 主な役割

- 共通ノード群を起動する
- `use_sim` に応じて実機系ノードまたはシミュレーション系ノードを起動する
- `robot_control_mode` を `r1_main_node` に渡す
- `r1_main_node` と `r1_chassis_control_node` の `cmd_vel_topic` を `/cmd_vel_target` に設定し、`r1_chassis_velocity_control_node` の `input_cmd_vel_topic` / `output_cmd_vel_topic` で補正経路を構成する
- LiDAR を使う実機構成では [`r1_slam.launch.py`](../launch/r1_slam.launch.py) を include する

## 引数

- `use_sim`
  - `false` で実機モード
  - `true` でシミュレーションモード
- `use_lidar`
  - 実機モード時に LiDAR 系構成を使うかを切り替える
- `robot_control_mode`
  - `manual` で `MANUAL/MODE1_DETECT_ORIGIN`
  - `auto` で `AUTO/ACT0`
- `use_aruco_display`
  - `false` で `r1_aruco_display_node` を起動しない
  - `true` で `r1_aruco_display_node` を起動する

## 主に起動するノード

- 常時起動
  - `joy_node`
  - `r1_main_node`
  - `r1_chassis_control_node`
  - `r1_chassis_velocity_control_node`
  - `r1_machine_manage_node`
  - 足回り・機構・Sabacan 関連ノード群
- `use_aruco_display:=true` のとき
  - `r1_aruco_display_node`
- 実機モード時
  - `eth2can_node`
  - `bno086_node`
  - `r1_odometry_node`
  - `r1_slam.launch.py` 内の localization 関連ノード
- シミュレーションモード時
  - [`r1_sim.launch.py`](../launch/r1_sim.launch.py) 内のダミーノード群

## 起動例

```bash
ros2 launch r1_bringup r1_bringup.launch.py
ros2 launch r1_bringup r1_bringup.launch.py robot_control_mode:=auto
ros2 launch r1_bringup r1_bringup.launch.py use_sim:=true
ros2 launch r1_bringup r1_bringup.launch.py use_aruco_display:=true
```
