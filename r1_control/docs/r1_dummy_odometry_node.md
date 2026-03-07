# r1_dummy_odometry_node

`r1_dummy_odometry_node` は、`/target_pose` を入力として簡易的なオドメトリ `/odometry` を生成するデバッグ用 ROS 2 ノードです。位置と姿勢は一次遅れフィルタでなめらかに追従し、更新周期は `timer_rate` パラメータで設定できます。

## トピック

- **Subscribe**
  - `/target_pose` (`geometry_msgs/msg/PoseStamped`): 追従先の目標姿勢。`pose.position.{x,y}` と `pose.orientation`（Yaw）を使用します。
- **Publish**
  - `/odometry` (`nav_msgs/msg/Odometry`): ダミーの自己位置。`header.frame_id = "odom"`、`child_frame_id = "base_link"` 固定です。

## 主なパラメータ

| パラメータ名 | 型 | デフォルト値 | 説明 |
| --- | --- | --- | --- |
| `timer_rate` | double | `100.0` | `/odometry` の更新レート [Hz]。内部の LPF 計算に使う `dt` も `1.0 / timer_rate` になります。 |
| `tau` | double | `0.5` | 一次遅れフィルタの時定数 [s]。大きいほど応答がゆっくりになります。 |

## 動作概要

1. `/target_pose` を受け取るたびに、最新の目標姿勢を内部に保存します。
2. `timer_rate` 周期のタイマで、目標位置 `x, y` と目標姿勢 `yaw` に対して一次遅れフィルタを適用します。
3. フィルタ後の姿勢を `nav_msgs/msg/Odometry` として `/odometry` に publish します。

## 起動例

```bash
source ~/ros2_ws/install/setup.bash
ros2 run r1_control r1_dummy_odometry_node --ros-args \
  -p timer_rate:=100.0 \
  -p tau:=0.5
```

目標姿勢を与える例:

```bash
ros2 topic pub --once /target_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'odom'},
  pose: {
    position: {x: 1.0, y: 0.5, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

## デバッグのヒント

- 出力確認: `ros2 topic echo /odometry`
- 追従を速くしたい場合: `tau` を小さくする
- 更新周期を変えたい場合: `timer_rate` を変更する
