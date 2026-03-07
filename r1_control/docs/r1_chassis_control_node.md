# r1_chassis_control_node

`r1_chassis_control_node` は、CSV から生成した軌道を起動時に `TrajectoryPlanner` で展開し、`/odometry` をフィードバックとして `TrajectoryFollower` で追従し、足回り速度指令 `/cmd_vel` を出力する ROS 2 ノードです。制御周期は `timer_rate`、可視化周期は `visualize_timer_rate` で設定します。

`PosFollower` も内部に生成されていますが、2026-03-07 時点の `timer_callback()` では単独の位置決め制御には使われておらず、`ACT0_START` 時の `reset()` のみ呼ばれています。

## トピック

- Subscribe
  - `/odometry` (`nav_msgs/msg/Odometry`): 自己位置と自己速度。
  - `/chassis_act_ref` (`std_msgs/msg/Int32`): 動作シーケンスの状態遷移指令。
- Publish
  - `/cmd_vel` (`geometry_msgs/msg/Twist`): シャーシ速度指令。
  - `/waypoints` (`nav_msgs/msg/Path`): 読み込んだ軌道のサンプル点列。ACT 開始時に 1 回 publish。
  - `/target_pose` (`geometry_msgs/msg/PoseStamped`): 現在追従している waypoint。可視化タイマで publish。
  - `/cmd_vel_arrow` (`visualization_msgs/msg/Marker`): 現在の `cmd_vel` を矢印 Marker で可視化。
  - `/robot_marker` (`visualization_msgs/msg/Marker`): 現在のロボット姿勢を CUBE Marker で可視化。
  - `/robot_trajectory` (`nav_msgs/msg/Path`): 動作実行中の実軌跡。
  - `/chassis_act_status` (`std_msgs/msg/Int32`): 現在の動作状態。

`/waypoints`、`/target_pose`、`/cmd_vel_arrow`、`/robot_marker`、`/robot_trajectory` の `frame_id` はすべて `odom` です。

## 動作概要

`/chassis_act_ref` で状態遷移指令を受けると、対応する軌道追従器を `reset()` し、`/waypoints` を publish して動作を開始します。動作中は制御タイマごとに `TrajectoryFollower::update()` を呼び、返ってきた waypoint を `/target_pose` に、速度指令を `/cmd_vel` に反映します。

最終 waypoint に到達し、位置・角度・継続時間の条件を満たすと、その動作は完了状態に遷移します。現在の状態は `/chassis_act_status` に publish されます。

## パラメータ

[`r1_machine_config.yaml`](/home/user/ros2_ws/src/gakurobo2026_r1/r1_bringup/config/r1_machine_config.yaml) に設定例があります。

| パラメータ名 | 型 | デフォルト値 | 説明 |
| --- | --- | --- | --- |
| `timer_rate` | double | `100.0` | 制御タイマ周期 [Hz]。`control_dt_ = 1.0 / timer_rate` として追従器にも渡されます。 |
| `visualize_timer_rate` | double | `10.0` | 可視化タイマ周期 [Hz]。 |
| `act_filebase` | string | `""` | 軌道 CSV のベースパス。`<act_filebase><index>_robot_parameter.csv` と `<act_filebase><index>_waypoints.csv` を読みます。 |
| `zone` | string | `"red"` | `"blue"` のとき waypoint の `x` 座標のみ符号反転します。`theta` は反転しません。 |
| `search_radius` | double | `0.0` | 現在位置から次の waypoint を探す半径 [m]。 |
| `kp_pos` | double | `0.0` | 並進位置誤差の P ゲイン。 |
| `ki_pos` | double | `0.0` | 並進位置誤差の I ゲイン。 |
| `kd_pos` | double | `0.0` | 軌道追従時は `(v_ref - v)` に掛かる D ゲイン、`PosFollower` では誤差差分に掛かる D ゲイン。 |
| `kff_pos` | double | `0.0` | 現在の `TrajectoryFollower::control()` では未使用です。 |
| `vel_limit` | double | `0.0` | `linear.x`, `linear.y` の出力上限 [m/s]。 |
| `kp_angle` | double | `0.0` | 角度誤差の P ゲイン。 |
| `ki_angle` | double | `0.0` | 角度誤差の I ゲイン。 |
| `kd_angle` | double | `0.0` | 軌道追従時は `(omega_ref - omega)` に掛かる D ゲイン、`PosFollower` では誤差差分に掛かる D ゲイン。 |
| `kff_angle` | double | `0.0` | 現在の `TrajectoryFollower::control()` では未使用です。 |
| `omega_limit` | double | `0.0` | `angular.z` の出力上限 [rad/s]。 |
| `goal_pos_range` | double | `0.0` | ゴール位置判定の閾値 [m]。 |
| `goal_angle_range` | double | `0.0` | ゴール角度判定の閾値 [rad]。 |
| `finish_time_threshold` | double | `0.0` | 位置・角度条件を満たし続ける必要がある時間 [s]。 |
| `publish_robot_trajectory_dist_threshold` | double | `0.1` | 実軌跡を追加 publish する最小移動距離 [m]。 |
| `publish_robot_trajectory_angle_threshold` | double | `5.0 * M_PI / 180.0` | 実軌跡を追加 publish する最小姿勢変化量 [rad]。 |
| `arrow_scale` | double | `0.2` | `cmd_vel_arrow` の長さスケール。 |

現行 YAML の主な設定値は以下です。

```yaml
r1_chassis_control_node:
  ros__parameters:
    act_filebase: "src/gakurobo2026_r1/data/"
    zone: "blue"
    timer_rate: 100.0
    visualize_timer_rate: 10.0
    search_radius: 0.2
    kp_pos: 0.7
    ki_pos: 0.0
    kd_pos: 0.5
    kff_pos: 0.0
    vel_limit: 2.0
    kp_angle: 0.4
    ki_angle: 0.0
    kd_angle: 0.5
    kff_angle: 0.0
    omega_limit: 3.14
    goal_pos_range: 0.05
    goal_angle_range: 0.05
    finish_time_threshold: 1.0
    arrow_scale: 3.0
```

注意:

- `publish_robot_trajectory_dist_threshold` と `publish_robot_trajectory_angle_threshold` は YAML に未記載ならコード既定値を使います。
- `act_filebase` が空、または CSV を開けない場合は起動時に Fatal で停止します。

## 入力ファイル

起動時に、実装で定義されている動作本数ぶんの軌道を読み込みます。

- ロボットパラメータ: `<act_filebase><index>_robot_parameter.csv`
- waypoint: `<act_filebase><index>_waypoints.csv`

例:

- `act_filebase := src/gakurobo2026_r1/data/`
  - `src/gakurobo2026_r1/data/0_robot_parameter.csv`, `src/gakurobo2026_r1/data/0_waypoints.csv`
  - `src/gakurobo2026_r1/data/1_robot_parameter.csv`, `src/gakurobo2026_r1/data/1_waypoints.csv`
  - `src/gakurobo2026_r1/data/2_robot_parameter.csv`, `src/gakurobo2026_r1/data/2_waypoints.csv`

### robot_parameter.csv

先頭 6 行を次の順番で読みます。

```csv
zone,blue
dt,0.01
v_max,5.0
a_max,5.0
j_max,10.0
omega_max,31.416
```

1 行目の `zone` は読み飛ばしです。このノードが実際に使うゾーン情報は ROS パラメータ `zone` です。

### waypoints.csv

各行は最大 4 列です。

```csv
x,y,theta,v_trans
```

- `x`, `y` は必須
- `theta`, `v_trans` は空欄可
- 空欄でない `theta`, `v_trans` だけが拘束条件として `TrajectoryPlanner::calc()` に渡されます

`TrajectoryPlanner` 側の制約により、始点と終点では `theta` と `v_trans` が必要になる前提でデータを作るのが安全です。

## 起動

bringup では [`r1_bringup.launch.py`](/home/user/ros2_ws/src/gakurobo2026_r1/r1_bringup/launch/r1_bringup.launch.py) から次のノード名で起動されます。

```bash
ros2 launch r1_bringup r1_bringup.launch.py
```

単体起動例:

```bash
ros2 run r1_control r1_chassis_control_node --ros-args \
  --params-file src/gakurobo2026_r1/r1_bringup/config/r1_machine_config.yaml
```

状態遷移指令の publish 例:

```bash
ros2 topic pub --once /chassis_act_ref std_msgs/msg/Int32 "{data: 1}"
```

## デバッグ

- `ros2 topic echo /chassis_act_status`
- `ros2 topic echo /cmd_vel`
- `ros2 topic echo /target_pose`
- `ros2 topic echo /waypoints`
- `ros2 topic echo /robot_trajectory`

`search_radius` が小さすぎると waypoint の進みが悪くなります。`zone == "blue"` では `x` だけ反転するため、軌道形状によっては向き制約との不整合が起きる点にも注意してください。
