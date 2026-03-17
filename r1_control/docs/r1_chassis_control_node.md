# r1_chassis_control_node

`r1_chassis_control_node` は、CSV から軌道を読み込んで内部で展開し、`/odometry` をフィードバックとして軌道追従を行い、シャーシ速度指令 `/cmd_vel` を出力する ROS 2 ノードです。可視化用の経路、目標姿勢、速度矢印、ロボットマーカ、実軌跡もあわせて publish します。

ACT の処理内容や状態遷移の詳細は今後変わる可能性があるため、このドキュメントでは扱いません。ここではノードの入出力、パラメータ、入力ファイル形式に限定して記載します。

## 役割

- 起動時に `<act_filebase><index>_robot_parameter.csv` と `<act_filebase><index>_waypoints.csv` を読み込み、`TrajectoryPlanner` で軌道を生成する
- `/chassis_act_ref` で指示された軌道に対して `TrajectoryFollower` を使い、`/odometry` に基づく軌道追従を行う
- 追従結果の速度指令を `/cmd_vel` に publish する
- 現在の制御状態を `/chassis_act_status` に publish する
- 可視化が有効なときに、経路や目標姿勢、実軌跡などを publish する

`PosFollower` も内部で生成されていますが、現状の制御ループでは主たる制御器としては使われていません。

## トピック

- Subscribe
  - `/odometry` (`nav_msgs/msg/Odometry`): 自己位置と自己速度
  - `/chassis_act_ref` (`std_msgs/msg/Int32`): 制御状態の入力
- Publish
  - `/cmd_vel` (`geometry_msgs/msg/Twist`): シャーシ速度指令
  - `/waypoints` (`nav_msgs/msg/Path`): 現在の軌道のサンプル点列
  - `/target_pose` (`geometry_msgs/msg/PoseStamped`): 現在追従中の目標姿勢
  - `/cmd_vel_arrow` (`visualization_msgs/msg/Marker`): 現在の `cmd_vel` の矢印可視化
  - `/robot_marker` (`visualization_msgs/msg/Marker`): ロボット本体の可視化
  - `/robot_trajectory` (`nav_msgs/msg/Path`): 実軌跡
  - `/chassis_error_tangent` (`std_msgs/msg/Float64`): 接線方向誤差
  - `/chassis_error_lateral` (`std_msgs/msg/Float64`): 法線方向誤差
  - `/chassis_error_theta` (`std_msgs/msg/Float64`): 角度誤差
  - `/chassis_act_status` (`std_msgs/msg/Int32`): 現在の制御状態

主な `frame_id` は以下です。

- `/waypoints`: `map`
- `/target_pose`: `map`
- `/robot_trajectory`: `map`
- `/cmd_vel_arrow`: `base_link`
- `/robot_marker`: `base_link`

## 動作概要

起動時に軌道 CSV を読み込み、各軌道に対応する `TrajectoryPlanner` と `TrajectoryFollower` を初期化します。制御ループでは `/odometry` を入力として追従器を更新し、返ってきた速度指令を `/cmd_vel` に反映します。

`TrajectoryFollower` は、現在位置に最も近い軌道サンプルを前方単調に探索し、そこから一定距離だけ先読みした点を追従目標として使います。先読み距離は `max(search_radius, v_trans * lookahead_time)` で決まり、通常時と終点近傍で別の PID ゲインを使います。位置誤差の PID は接線方向と法線方向で独立に計算し、終点で位置・姿勢誤差が閾値内に入り、その状態が `finish_time_threshold` 秒以上継続したときに追従完了と判定します。

並進の feedforward は「現在位置から目標点へ向かう向き」ではなく、目標点近傍の軌道接線方向に合わせて生成されます。現状はシンプルな片側差分で接線方向を求めています。位置誤差の PID も接線方向・法線方向に分解して計算し、最後に `odom` 座標系の `x`, `y` 指令へ戻しています。

`use_map == true` のときは、軌道上の目標姿勢を `map` から `odom` へ TF 変換してから追従計算に使います。`false` のときは軌道データをそのまま `odom` 系として扱います。

可視化ループでは、必要に応じて現在の目標姿勢、速度矢印、ロボットマーカ、実軌跡を publish します。実軌跡は `odom` から `map` へ TF 変換した姿勢を蓄積しており、TF が取得できない場合はその周期の追加をスキップします。

## パラメータ

設定例は [`r1_machine_config.yaml`](/home/user/ros2_ws/src/gakurobo2026_r1/r1_bringup/config/r1_machine_config.yaml) にあります。

| パラメータ名 | 型 | デフォルト値 | 説明 |
| --- | --- | --- | --- |
| `timer_rate` | double | `100.0` | 制御タイマ周期 [Hz]。`control_dt_ = 1.0 / timer_rate` として追従器へ渡されます。 |
| `visualize_timer_rate` | double | `10.0` | 可視化タイマ周期 [Hz]。 |
| `act_filebase` | string | `""` | 軌道 CSV のベースパス。`<act_filebase><index>_robot_parameter.csv` と `<act_filebase><index>_waypoints.csv` を読みます。 |
| `zone` | string | `"red"` | `"blue"` のとき waypoint の `x` 座標だけを符号反転します。 |
| `use_map` | bool | `true` | `true` のとき軌道上の目標姿勢を `map` から `odom` へ TF 変換して追従します。 |
| `search_radius` | double | `0.0` | 先読み距離の下限 [m]。低速時でも目標点が近すぎないようにするために使います。 |
| `lookahead_time` | double | `0.3` | 速度に応じて先読み距離を伸ばすための時間 [s]。先読み距離は `v_trans * lookahead_time` で計算されます。 |
| `kp_pos_tangent_usual` | double | `0.0` | 通常時の接線方向位置誤差の P ゲイン。 |
| `ki_pos_tangent_usual` | double | `0.0` | 通常時の接線方向位置誤差の I ゲイン。 |
| `kd_pos_tangent_usual` | double | `0.0` | 通常時の接線方向速度誤差の D ゲイン。 |
| `kp_pos_tangent_goal` | double | `0.0` | 終点近傍で使う接線方向位置誤差の P ゲイン。 |
| `ki_pos_tangent_goal` | double | `0.0` | 終点近傍で使う接線方向位置誤差の I ゲイン。 |
| `kd_pos_tangent_goal` | double | `0.0` | 終点近傍で使う接線方向速度誤差の D ゲイン。 |
| `kp_pos_normal_usual` | double | `0.0` | 通常時の法線方向位置誤差の P ゲイン。 |
| `ki_pos_normal_usual` | double | `0.0` | 通常時の法線方向位置誤差の I ゲイン。 |
| `kd_pos_normal_usual` | double | `0.0` | 通常時の法線方向速度誤差の D ゲイン。 |
| `kp_pos_normal_goal` | double | `0.0` | 終点近傍で使う法線方向位置誤差の P ゲイン。 |
| `ki_pos_normal_goal` | double | `0.0` | 終点近傍で使う法線方向位置誤差の I ゲイン。 |
| `kd_pos_normal_goal` | double | `0.0` | 終点近傍で使う法線方向速度誤差の D ゲイン。 |
| `vel_i_limit` | double | `0.0` | `linear.x`, `linear.y` の積分項出力上限 [m/s]。 |
| `vel_output_limit` | double | `0.0` | `linear.x`, `linear.y` の指令出力上限 [m/s]。 |
| `kp_angle_usual` | double | `0.0` | 通常時の角度誤差の P ゲイン。 |
| `ki_angle_usual` | double | `0.0` | 通常時の角度誤差の I ゲイン。 |
| `kd_angle_usual` | double | `0.0` | 通常時の角速度誤差の D ゲイン。 |
| `kp_angle_goal` | double | `0.0` | 終点近傍で使う角度誤差の P ゲイン。 |
| `ki_angle_goal` | double | `0.0` | 終点近傍で使う角度誤差の I ゲイン。 |
| `kd_angle_goal` | double | `0.0` | 終点近傍で使う角速度誤差の D ゲイン。 |
| `omega_i_limit` | double | `0.0` | `angular.z` の積分項出力上限 [rad/s]。 |
| `omega_output_limit` | double | `0.0` | `angular.z` の指令出力上限 [rad/s]。 |
| `goal_pos_range` | double | `0.0` | ゴール位置判定の閾値 [m]。 |
| `goal_angle_range` | double | `0.0` | ゴール角度判定の閾値 [rad]。 |
| `finish_time_threshold` | double | `0.0` | ゴール条件を満たし続ける必要がある時間 [s]。 |
| `publish_robot_trajectory_dist_threshold` | double | `0.1` | 実軌跡を追加する最小移動距離 [m]。 |
| `publish_robot_trajectory_angle_threshold` | double | `5.0 * M_PI / 180.0` | 実軌跡を追加する最小姿勢変化量 [rad]。 |
| `enable_visualization` | bool | `true` | `false` のとき可視化タイマは何も publish しません。 |
| `arrow_scale` | double | `0.2` | `cmd_vel_arrow` の長さスケール。 |

注意:

- `act_filebase` が空、または CSV を開けない場合は起動時に Fatal で停止します。
- `publish_robot_trajectory_dist_threshold`、`publish_robot_trajectory_angle_threshold`、`enable_visualization`、`arrow_scale` は YAML に未記載ならコード既定値を使います。
- `zone == "blue"` では `x` だけ反転し、`theta` は反転しません。

## 入力ファイル

起動時に、実装で確保されている軌道本数ぶんのファイルを読み込みます。

- ロボットパラメータ: `<act_filebase><index>_robot_parameter.csv`
- waypoint: `<act_filebase><index>_waypoints.csv`

例:

- `act_filebase := src/gakurobo2026_r1/data/`
  - `src/gakurobo2026_r1/data/0_robot_parameter.csv`
  - `src/gakurobo2026_r1/data/0_waypoints.csv`
  - `src/gakurobo2026_r1/data/1_robot_parameter.csv`
  - `src/gakurobo2026_r1/data/1_waypoints.csv`
  - `src/gakurobo2026_r1/data/2_robot_parameter.csv`
  - `src/gakurobo2026_r1/data/2_waypoints.csv`

### robot_parameter.csv

先頭 6 行を次の順番で読みます。7 行目以降は無視されます。

```csv
zone,blue
dt,0.01
v_max,5.0
a_max,5.0
j_max,10.0
omega_max,31.416
```

1 行目の `zone` は読み飛ばしです。ノードが実際に使うゾーン情報は ROS パラメータ `zone` です。

### waypoints.csv

各行は最大 4 列です。

```csv
x,y,theta,v_trans
```

- `x`, `y` は必須
- `theta`, `v_trans` は空欄可
- 空欄でない `theta`, `v_trans` だけが拘束条件として `TrajectoryPlanner::calc()` に渡されます
- ヘッダ行は想定されていません。すべての行がデータとして読み込まれます

## 起動

bringup では [`r1_bringup.launch.py`](/home/user/ros2_ws/src/gakurobo2026_r1/r1_bringup/launch/r1_bringup.launch.py) から起動されます。

```bash
ros2 launch r1_bringup r1_bringup.launch.py
```

単体起動例:

```bash
ros2 run r1_control r1_chassis_control_node --ros-args \
  --params-file src/gakurobo2026_r1/r1_bringup/config/r1_machine_config.yaml
```

## デバッグ

- `ros2 topic echo /chassis_act_status`
- `ros2 topic echo /cmd_vel`
- `ros2 topic echo /chassis_error_tangent`
- `ros2 topic echo /chassis_error_lateral`
- `ros2 topic echo /chassis_error_theta`
- `ros2 topic echo /target_pose`
- `ros2 topic echo /waypoints`
- `ros2 topic echo /robot_trajectory`

`enable_visualization == false` のときは `/target_pose`、`/cmd_vel_arrow`、`/robot_marker`、`/robot_trajectory` は出ません。`map` と `odom` の TF が未接続だと `/robot_trajectory` の更新はスキップされ、`use_map == true` の場合は追従中の目標姿勢変換にも影響します。
