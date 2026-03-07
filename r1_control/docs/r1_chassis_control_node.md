# r1_chassis_control_node

`r1_chassis_control_node` は、CSV から生成した軌道（waypoint 列＋拘束条件）を `TrajectoryPlanner` で事前計算し、`/odometry` をフィードバックとして `TrajectoryFollower` により追従させ、足回り速度指令 `/cmd_vel` を出力する ROS 2 ノードです。制御周期は `timer_rate`、可視化系の周期は `visualize_timer_rate` でそれぞれ設定でき、デフォルトは制御 100 Hz・可視化 20 Hz です。

現状の実装では ACT（動作シーケンス）は 3 個（`ACT_N = 3`、ACT0/ACT1/ACT2）です。

## トピック

- **Subscribe**
  - `/odometry` (`nav_msgs/msg/Odometry`): 現在の自己位置。`pose.pose.position.{x,y}` と `pose.pose.orientation`（Yaw）を追従に使用します。
  - `/chassis_act_ref` (`std_msgs/msg/Int32`): ACT 参照値（状態遷移指令）。
- **Publish**
  - `/cmd_vel` (`geometry_msgs/msg/Twist`): 足回り速度指令（`linear.{x,y}` と `angular.z`）。
  - `/waypoints` (`nav_msgs/msg/Path`): デバッグ用に軌道サンプル点列を publish（`header.frame_id = "odom"`）。
  - `/target_pose` (`geometry_msgs/msg/PoseStamped`): 現在追従中の waypoint を publish（`header.frame_id = "odom"`）。
  - `/robot_marker` (`visualization_msgs/msg/Marker`): デバッグ用にロボット位置の Marker（`header.frame_id = "odom"`）を周期 publish。
  - `/chassis_act_status` (`std_msgs/msg/Int32`): 現在の ACT 状態を周期 publish。

注意: `/waypoints` / `/target_pose` / `/robot_marker` は `odom` フレームで publish されています（利用側でフレーム整合に注意してください）。

## 主なパラメータ

すべて起動時に読み込まれます（実行中の動的更新コールバックは未実装）。
`kp_*` / `kff_*` は **位置（`x,y`）** と **角度（Yaw）** で分割しています（旧 `kp` / `kff` は廃止）。

| パラメータ名 | 型 | デフォルト値 | 説明 |
| --- | --- | --- | --- |
| `timer_rate` | double | `100.0` | 制御ループと `/chassis_act_status` の周期 publish に使う更新レート [Hz]。内部の追従器に渡す `dt` も `1.0 / timer_rate` になります。 |
| `visualize_timer_rate` | double | `20.0` | `/target_pose`、`/robot_marker`、`/robot_trajectory` の可視化 publish に使う更新レート [Hz]。 |
| `act_filebase` | string | `""` | 入力 CSV のベースパス。`<act_filebase><n>_robot_parameter.csv` と `<act_filebase><n>_waypoints.csv` を読みます（例は後述）。空の場合は起動時に Fatal で停止します。 |
| `zone` | string | `"red"` | `"blue"` の場合、読み込んだ waypoint の `x` 座標を反転して軌道生成します（ゾーン対称対応）。 |
| `search_radius` | double | `0.0` | 次の waypoint 探索の半径 [m]。現在位置からの距離がこの半径より大きい最初の点を「次の点」とみなします。 |
| `kp_pos` | double | `0.0` | 位置（`x,y`）誤差に対する P ゲイン。 |
| `ki_pos` | double | `0.0` | 位置制御の I ゲイン（コード上は保持していますが、現状の制御計算では未使用）。 |
| `kd_pos` | double | `0.0` | 位置制御の D ゲイン（同上、未使用）。 |
| `kff_pos` | double | `0.0` | 位置制御の軌道フィードフォワード係数。`v_ref`（`v_trans` から作る参照速度 `vx_ref, vy_ref`）に対して乗算されます。 |
| `kp_angle` | double | `0.0` | 角度（Yaw）誤差に対する P ゲイン。 |
| `ki_angle` | double | `0.0` | 角度制御の I ゲイン（コード上は保持していますが、現状の制御計算では未使用）。 |
| `kd_angle` | double | `0.0` | 角度制御の D ゲイン（同上、未使用）。 |
| `kff_angle` | double | `0.0` | 角度制御の軌道フィードフォワード係数。`v_ref`（軌道の `omega_ref`）に対して乗算されます。 |
| `goal_pos_range` | double | `0.0` | ゴール判定（位置）の閾値 [m]。最終点で「距離 < goal_pos_range」で位置収束とみなします。 |
| `goal_angle_range` | double | `0.0` | ゴール判定（角度）の閾値 [rad]。最終点で「角度差 < goal_angle_range」で角度収束とみなします。 |
| `finish_time_threshold` | double | `0.0` | 収束判定の継続時間 [s]。最終点の「距離・角度」が閾値内の状態がこの時間以上続いた場合に終了扱いになります（`0.0` だと即時判定）。 |

## ACT（状態遷移）

`/chassis_act_ref` で受け取った値に応じて `timer_rate` 周期の制御タイマ内で状態遷移します（同時に `/chassis_act_status` に現在状態を publish）。`/target_pose`、`/robot_marker`、`/robot_trajectory` は別の `visualize_timer_rate` 周期タイマで publish されます。

- `0` (`ACT_NONE`): 何もしない
- `1` (`ACT0_START`): ACT0 開始要求
- `2` (`ACT0`): ACT0 実行中
- `3` (`ACT0_FINISH`): ACT0 完了
- `11` (`ACT1_START`): ACT1 開始要求
- `12` (`ACT1`): ACT1 実行中
- `13` (`ACT1_FINISH`): ACT1 完了
- `21` (`ACT2_START`): ACT2 開始要求
- `22` (`ACT2`): ACT2 実行中
- `23` (`ACT2_FINISH`): ACT2 完了

動作概要:

1. `ACT*_START` を受け取ると内部で `ACT*` に遷移し、追従器を `reset()`、デバッグ用に `/waypoints` を 1 回 publish します。
2. `ACT*` の間は、`/odometry` を用いて追従器 `update()` を回し、`/cmd_vel` と `/target_pose` を周期 publish します。
3. 追従器が終了判定（最終点＋閾値内、かつ `finish_time_threshold` の条件を満たす）になると `ACT*_FINISH` に遷移します。

## 入力ファイル（CSV）

ノード起動時に ACT 数（現状 3 個）ぶんの軌道を読み込み・計算します（ACT0〜ACT2）。ファイル名は以下の規則です。

- ロボットパラメータ: `<act_filebase><n>_robot_parameter.csv`
- waypoint: `<act_filebase><n>_waypoints.csv`

例:

- `act_filebase:=/home/user/ros2_ws/trajectory` の場合
  - ACT0: `/home/user/ros2_ws/trajectory0_robot_parameter.csv`, `/home/user/ros2_ws/trajectory0_waypoints.csv`
  - ACT1: `/home/user/ros2_ws/trajectory1_robot_parameter.csv`, `/home/user/ros2_ws/trajectory1_waypoints.csv`
  - ACT2: `/home/user/ros2_ws/trajectory2_robot_parameter.csv`, `/home/user/ros2_ws/trajectory2_waypoints.csv`

### `<n>_robot_parameter.csv` 形式

先頭から以下の 6 行を読み取ります（7 行目以降は無視されます）。

```
zone,blue            # 1行目: 読み捨て（このノードでは参照しない）
dt,0.01              # 制御周期 [s]（軌道生成側のサンプリング周期）
v_max,5.0            # 最大並進速度 [m/s]
a_max,5.0            # 最大並進加速度 [m/s^2]
j_max,10.0           # 最大並進躍度 [m/s^3]
omega_max,31.416     # 最大角速度 [rad/s]（超過チェック用）
```

注意:

- このノードの追従制御周期は `1.0 / timer_rate` [s] です。CSV の `dt` と大きくずれると追従挙動に差が出るため、できるだけ整合する値を設定してください。

### `<n>_waypoints.csv` 形式

各行が 1 waypoint で、カンマ区切りで最大 4 列です。

```
x, y, theta, v_trans
```

- `x, y` は必須（単位は [m] を想定）
- `theta`（[rad]）と `v_trans`（並進速度 [m/s]）は「任意」ですが、軌道生成（`TrajectoryPlanner.calc()`）側の入力チェックにより **始点（index=0）と終点（index=最後）には必須** です（中間点は空欄可）
- 空欄は「拘束条件なし」として扱われ、内部ではその行の `theta_wp / v_trans_wp` には追加されません

## 起動例

```bash
source ~/ros2_ws/install/setup.bash
ros2 run r1_control r1_chassis_control_node --ros-args \
  -p timer_rate:=100.0 \
  -p visualize_timer_rate:=20.0 \
  -p act_filebase:=/home/user/ros2_ws/trajectory \
  -p search_radius:=0.2 \
  -p kp_pos:=1.0 -p kff_pos:=0.75 \
  -p kp_angle:=1.0 -p kff_angle:=0.75 \
  -p goal_pos_range:=0.05 -p goal_angle_range:=0.05
```

ACT0 を開始する例:

```bash
ros2 topic pub --once /chassis_act_ref std_msgs/msg/Int32 "{data: 1}"
```

## デバッグのヒント

- 軌道（点列）の確認: `ros2 topic echo /waypoints`
- 追従ターゲットの確認: `ros2 topic echo /target_pose`
- 速度指令の確認: `ros2 topic echo /cmd_vel`
- 状態遷移の確認: `ros2 topic echo /chassis_act_status`

`search_radius = 0.0` の場合、開始直後に現在位置が先頭 waypoint と一致していないと「次の点」へ進みにくくなるため、実機では 0.1〜0.3[m] 程度を目安に調整してください。
