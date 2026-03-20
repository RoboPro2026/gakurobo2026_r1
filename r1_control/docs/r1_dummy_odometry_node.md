# r1_dummy_odometry_node

`r1_dummy_odometry_node` は、`/target_pose` を入力として簡易的なオドメトリ `/odometry` を生成するシミュレーション用 ROS 2 ノードです。位置と姿勢は一次遅れフィルタでなめらかに追従し、`map -> odom` と `odom -> base_link` TF を publish します。

`r1_bringup.launch.py` を `use_sim:=true` で起動したときに使う前提のノードです。実機モードでは起動しません。`/map` 自体は `nav2_map_server` が publish します。

## トピック

- Subscribe
  - `/target_pose` (`geometry_msgs/msg/PoseStamped`): 追従先の目標姿勢。`pose.position.{x,y}` と `pose.orientation` の Yaw を使用します。
  - `/initialpose` (`geometry_msgs/msg/PoseWithCovarianceStamped`): ダミー自己位置のリセット用。受信した姿勢を現在位置として即座に反映します。
- Publish
  - `/odometry` (`nav_msgs/msg/Odometry`): ダミーの自己位置。`header.frame_id = "odom"`、`child_frame_id = "base_link"` 固定です。

## TF

- `map -> odom`: 固定 TF を publish
- `odom -> base_link`: ダミーオドメトリに対応した TF を publish

## 主なパラメータ

| パラメータ名 | 型 | デフォルト値 | 説明 |
| --- | --- | --- | --- |
| `timer_rate` | double | `100.0` | `/odometry` の更新レート [Hz]。内部の LPF 計算に使う `dt` も `1.0 / timer_rate` になります。 |
| `tau` | double | `0.5` | 一次遅れフィルタの時定数 [s]。大きいほど応答がゆっくりになります。 |

## 動作概要

1. `/target_pose` を受け取るたびに、最新の目標姿勢を内部に保存します。
2. `timer_rate` 周期のタイマで、目標位置 `x, y` と目標姿勢 `yaw` に対して一次遅れフィルタを適用します。
3. フィルタ後の姿勢を `nav_msgs/msg/Odometry` として `/odometry` に publish します。
4. 同じ姿勢を `odom -> base_link` TF として publish します。
5. 起動時に `map -> odom` の固定 TF を publish します。

`/initialpose` を受け取ったときは、現在位置と内部 LPF 状態をその姿勢へ即座に揃えます。

## 起動例

単体起動:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run r1_control r1_dummy_odometry_node --ros-args \
  -p timer_rate:=100.0 \
  -p tau:=0.5
```

bringup のシミュレーションモード起動:

```bash
ros2 launch r1_bringup r1_bringup.launch.py use_sim:=true
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

`/initialpose` で位置をリセットする例:

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
  header: {frame_id: 'map'},
  pose: {
    pose: {
      position: {x: 0.0, y: 0.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  }
}"
```

## デバッグのヒント

- 出力確認: `ros2 topic echo /odometry`
- 地図確認: `ros2 topic echo /map --once`
  - `use_sim:=true` のときは `map_server` が publish します。
- TF 確認: `ros2 run tf2_ros tf2_echo map odom`
- TF 確認: `ros2 run tf2_ros tf2_echo odom base_link`
- 追従を速くしたい場合: `tau` を小さくする
- 更新周期を変えたい場合: `timer_rate` を変更する
