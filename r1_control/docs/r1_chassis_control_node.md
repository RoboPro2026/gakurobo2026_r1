# r1_chassis_control_node

`r1_chassis_control_node` は、CSV から生成した軌道（waypoint 列＋拘束条件）を `TrajectoryPlanner` で事前計算し、`/odometry` をフィードバックとして `TrajectoryFollower` により追従させ、足回り速度指令 `/cmd_vel` を出力する ROS 2 ノードです。制御周期は 10 ms（wall timer）です。

現状の実装では ACT（動作シーケンス）は 1 個（`ACT_N = 1`、ACT0 のみ）に固定されています。

## トピック

- **Subscribe**
  - `/odometry` (`nav_msgs/msg/Odometry`): 現在の自己位置。`pose.pose.position.{x,y}` と `pose.pose.orientation`（Yaw）を追従に使用します。
  - `/chassis_act_ref` (`std_msgs/msg/Int32`): ACT 参照値（状態遷移指令）。
- **Publish**
  - `/cmd_vel` (`geometry_msgs/msg/Twist`): 足回り速度指令（`linear.{x,y}` と `angular.z`）。
  - `/waypoints` (`nav_msgs/msg/Path`): デバッグ用に軌道サンプル点列を publish（`header.frame_id = "odom"`）。
  - `/target_pose` (`geometry_msgs/msg/PoseStamped`): 現在追従中の waypoint を publish（`header.frame_id = "map"`）。
  - `/robot_marker` (`visualization_msgs/msg/Marker`): Publisher は作成されていますが、現状このノード単体では publish していません。
  - `/chassis_act_status` (`std_msgs/msg/Int32`): 現在の ACT 状態を周期 publish。

注意: `/odometry` は通常 `odom` フレーム、`/target_pose` は `map` フレーム固定で publish されています（フレーム整合は利用側で注意してください）。

## 主なパラメータ

すべて起動時に読み込まれます（実行中の動的更新コールバックは未実装）。

| パラメータ名 | 型 | デフォルト値 | 説明 |
| --- | --- | --- | --- |
| `act_filebase` | string | `""` | 入力 CSV のベースパス。`<act_filebase><n>_robot_parameter.csv` と `<act_filebase><n>_waypoints.csv` を読みます（例は後述）。空の場合は起動時に Fatal で停止します。 |
| `zone` | string | `"red"` | 現状このノードでは未使用（将来のゾーン切替等の名残）。 |
| `search_radius` | double | `0.0` | 次の waypoint 探索の半径 [m]。現在位置からの距離がこの半径より大きい最初の点を「次の点」とみなします。 |
| `kp` | double | `0.0` | 位置誤差に対する P ゲイン（`x,y,theta` 共通で同じ値を使用）。 |
| `ki` | double | `0.0` | I ゲイン（コード上は保持していますが、現状の制御計算では未使用）。 |
| `kd` | double | `0.0` | D ゲイン（同上、未使用）。 |
| `kff` | double | `0.0` | 軌道フィードフォワード係数。`v_ref`（`v_trans` と `omega` から作る参照速度）に対して乗算されます。 |
| `goal_range` | double | `0.0` | ゴール判定の閾値。最終点で「距離 < goal_range かつ 角度差 < goal_range」で終了扱いになります。 |

## ACT（状態遷移）

`/chassis_act_ref` で受け取った値に応じて 10ms タイマ内で状態遷移します（同時に `/chassis_act_status` に現在状態を publish）。

- `0` (`ACT_NONE`): 何もしない
- `10` (`ACT0_START`): ACT0 開始要求
- `11` (`ACT0`): ACT0 実行中
- `12` (`ACT0_FINISH`): ACT0 完了

動作概要:

1. `ACT0_START (10)` を受け取ると内部で `ACT0 (11)` に遷移し、追従器を `reset()`、デバッグ用に `/waypoints` を 1 回 publish します。
2. `ACT0 (11)` の間は、`/odometry` を用いて追従器 `update()` を回し、`/cmd_vel` と `/target_pose` を周期 publish します。
3. 追従器が終了判定（最終点＋閾値内）になると `ACT0_FINISH (12)` に遷移します。

## 入力ファイル（CSV）

ノード起動時に ACT 数（現状 1 個）ぶんの軌道を読み込み・計算します。ファイル名は以下の規則です。

- ロボットパラメータ: `<act_filebase><n>_robot_parameter.csv`
- waypoint: `<act_filebase><n>_waypoints.csv`

例:

- `act_filebase:=/home/user/ros2_ws/trajectory` の場合
  - ACT0: `/home/user/ros2_ws/trajectory0_robot_parameter.csv`, `/home/user/ros2_ws/trajectory0_waypoints.csv`

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

- このノードの追従制御周期は 10ms 固定（`TrajectoryFollower::set_param(..., dt=0.01, ...)`）です。`dt` と整合する値を CSV 側にも設定してください。

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
  -p act_filebase:=/home/user/ros2_ws/trajectory \
  -p search_radius:=0.2 \
  -p kp:=1.0 -p kff:=1.0 \
  -p goal_range:=0.05
```

ACT0 を開始する例:

```bash
ros2 topic pub --once /chassis_act_ref std_msgs/msg/Int32 "{data: 10}"
```

## デバッグのヒント

- 軌道（点列）の確認: `ros2 topic echo /waypoints`
- 追従ターゲットの確認: `ros2 topic echo /target_pose`
- 速度指令の確認: `ros2 topic echo /cmd_vel`
- 状態遷移の確認: `ros2 topic echo /chassis_act_status`

`search_radius = 0.0` の場合、開始直後に現在位置が先頭 waypoint と一致していないと「次の点」へ進みにくくなるため、実機では 0.1〜0.3[m] 程度を目安に調整してください。

