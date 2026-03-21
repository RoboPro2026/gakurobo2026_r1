# r1_dummy_odometry_node

`r1_dummy_odometry_node` は、`/cmd_vel` と `Path` を入力として簡易的なオドメトリ `/odometry` を生成するシミュレーション用 ROS 2 ノードです。現在の既定動作では `/waypoints` の `nav_msgs/msg/Path` を subscription し、その軌道上を `cmd_vel` の並進速度で進みます。`cmd_vel` は速度として使い、位置そのものは Path 上へ拘束します。Path が無い場合だけ、30 kg のロボットを想定した簡易ダイナミクスモデルへフォールバックします。

`r1_bringup.launch.py` を `use_sim:=true` で起動したときに使う前提のノードです。実機モードでは起動しません。`/map` 自体は `nav2_map_server` が publish します。

## トピック

- Subscribe
  - `/cmd_vel` (`geometry_msgs/msg/Twist`): ダミーオドメトリの入力速度指令です。`linear.x`, `linear.y`, `angular.z` を使用します。
  - `/waypoints` (`nav_msgs/msg/Path`): 追従する軌道です。既定ではこの Path 上を `cmd_vel` の並進速度に応じて進みます。
  - `/target_pose` (`geometry_msgs/msg/PoseStamped`): 積分ドリフト補正用の目標姿勢です。`enable_target_pose_correction` が true のとき、一定周期ごとにこの姿勢との差を判定し、閾値を超えていれば現在位置をこの姿勢へ直接合わせます。
  - `/initialpose` (`geometry_msgs/msg/PoseWithCovarianceStamped`): ダミー自己位置のリセット用です。受信した姿勢を現在位置として即座に反映します。
  - `/set_odometry` (`std_msgs/msg/Float64MultiArray`): `r1_main_node` 互換の自己位置リセット用です。`[x, y, yaw]` の 3 要素を受け取ります。
- Publish
  - `/odometry` (`nav_msgs/msg/Odometry`): ダミーの自己位置です。`header.frame_id = "odom"`、`child_frame_id = "base_link"` 固定です。

## TF

- `map -> odom`: 固定 TF を publish
- `odom -> base_link`: ダミーオドメトリに対応した TF を publish

## 主なパラメータ

| パラメータ名 | 型 | デフォルト値 | 説明 |
| --- | --- | --- | --- |
| `timer_rate` | double | `100.0` | `/odometry` の更新レート [Hz]。内部積分の `dt` は `1.0 / timer_rate` です。 |
| `tau` | double | `0.15` | `/cmd_vel` に対する一次遅れフィルタの時定数 [s]。大きいほど指令追従がゆっくりになります。 |
| `mass` | double | `30.0` | ロボット質量 [kg]。`max_force_x`, `max_force_y` から最大加速度を計算するのに使います。 |
| `yaw_inertia` | double | `1.8` | yaw 軸まわりの慣性モーメントです。`max_torque_z` から最大角加速度を計算するのに使います。 |
| `linear_velocity_response_gain` | double | `6.0` | 並進速度の追従ゲインです。大きいほど速度指令へ素早く近づきます。 |
| `angular_velocity_response_gain` | double | `8.0` | 角速度の追従ゲインです。大きいほど角速度指令へ素早く近づきます。 |
| `max_force_x` | double | `120.0` | x 方向の最大推力 [N] です。 |
| `max_force_y` | double | `120.0` | y 方向の最大推力 [N] です。 |
| `max_torque_z` | double | `6.0` | z 軸まわりの最大トルク [Nm] です。 |
| `max_jerk_x` | double | `40.0` | x 方向加速度の変化率上限です。大きいほど応答が鋭くなります。 |
| `max_jerk_y` | double | `40.0` | y 方向加速度の変化率上限です。 |
| `max_jerk_z` | double | `30.0` | yaw 角加速度の変化率上限です。 |
| `use_path_tracking` | bool | `true` | `true` のときは `/waypoints` を優先し、Path 上を `cmd_vel` の速度で進みます。 |
| `enable_target_pose_correction` | bool | `true` | `/target_pose` による位置補正を有効にするかどうかです。 |
| `target_pose_tracking_tau` | double | `0.08` | `/target_pose` へ連続追従するときの時定数 [s] です。小さいほど目標姿勢へ強く追従します。 |
| `target_pose_correction_period` | double | `0.1` | 目標姿勢への補正を行う周期 [s] です。 |
| `target_pose_position_snap_threshold` | double | `0.03` | `/target_pose` との位置誤差がこの値を超えたときに現在位置を直接補正します。 |
| `target_pose_yaw_snap_threshold` | double | `0.052` | `/target_pose` との yaw 誤差がこの値を超えたときに姿勢を直接補正します。 |

## 動作概要

1. `/cmd_vel` を受け取るたびに、最新の速度指令を内部に保存します。
2. `/waypoints` を受け取ると、Path 上の累積距離を内部で計算します。
3. `timer_rate` 周期のタイマで、`linear.x`, `linear.y`, `angular.z` に一次遅れフィルタを適用します。
4. `use_path_tracking = true` かつ Path があるときは、`cmd_vel` の並進速度の大きさで Path 上の進捗距離を進め、現在位置をその Path 上の点へ設定します。
5. このとき姿勢も Path の向きに合わせ、`/odometry` の速度成分は `cmd_vel` から計算します。
6. Path が無いときだけ、30 kg モデルと jerk 制限付きの簡易ダイナミクスへフォールバックします。
7. `enable_target_pose_correction` が true のときは、フォールバックモードでのみ `/target_pose` を使った補正を行います。
8. 補正後の姿勢を `nav_msgs/msg/Odometry` として `/odometry` に publish します。
9. 同じ姿勢を `odom -> base_link` TF として publish します。
10. 起動時に `map -> odom` の固定 TF を publish します。

`/initialpose` または `/set_odometry` を受け取ったときは、現在位置を即座に更新し、内部の速度状態もゼロへリセットします。

## 起動例

単体起動:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run r1_control r1_dummy_odometry_node --ros-args \
  -p timer_rate:=100.0 \
  -p mass:=30.0 \
  -p tau:=0.15
```

bringup のシミュレーションモード起動:

```bash
ros2 launch r1_bringup r1_bringup.launch.py use_sim:=true
```

速度指令を与える例:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{
  linear: {x: 0.3, y: 0.0, z: 0.0},
  angular: {x: 0.0, y: 0.0, z: 0.2}
}"
```

補正用の目標姿勢を与える例:

```bash
ros2 topic pub --once /target_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: -5.0, y: 1.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

`/initialpose` で位置をリセットする例:

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
  header: {frame_id: 'map'},
  pose: {
    pose: {
      position: {x: -5.5, y: 0.5, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  }
}"
```

`/set_odometry` で位置をリセットする例:

```bash
ros2 topic pub --once /set_odometry std_msgs/msg/Float64MultiArray "{
  data: [-5.5, 0.5, 0.0]
}"
```

## デバッグのヒント

- 出力確認: `ros2 topic echo /odometry`
- 速度確認: `ros2 topic echo /cmd_vel`
- Path 確認: `ros2 topic echo /waypoints`
- 補正目標確認: `ros2 topic echo /target_pose`
- 地図確認: `ros2 topic echo /map --once`
  - `use_sim:=true` のときは `map_server` が publish します。
- TF 確認: `ros2 run tf2_ros tf2_echo map odom`
- TF 確認: `ros2 run tf2_ros tf2_echo odom base_link`
- 応答を速くしたい場合: `tau` を小さくするか、`linear_velocity_response_gain` と `angular_velocity_response_gain` を上げる
- 重い感じを弱めたい場合: `max_force_x`, `max_force_y`, `max_torque_z` を上げる
- ガタつきを減らしたい場合: `max_jerk_x`, `max_jerk_y`, `max_jerk_z` を下げる
- 目標姿勢へもっと強く追従したい場合: `target_pose_tracking_tau` を小さくする
- 補正頻度を上げたい場合: `target_pose_correction_period` を小さくする
- 補正を少し鈍くしたい場合: `target_pose_position_snap_threshold` と `target_pose_yaw_snap_threshold` を大きくする
- 更新周期を変えたい場合: `timer_rate` を変更する
