# r1_dummy_map_node

`r1_dummy_map_node` は、簡易 localization として `map -> odom` TF を publish する ROS 2 ノードです。起動直後は `map` と `odom` を一致させ、`/initialpose` を受けると現在の `odom -> base_link` を基準に `map -> odom` を再計算します。

`r1_dummy_odometry_node` が `odom -> base_link` を担当し、このノードが `map -> odom` を担当することで、ダミーオドメトリと地図側の責務を分離しています。

## トピック

- Subscribe
  - `/initialpose` (`geometry_msgs/msg/PoseWithCovarianceStamped`): `map` 座標系での目標自己位置です。現在の `odom -> base_link` と組み合わせて `map -> odom` を更新します。
- Publish
  - TF `map -> odom`

## 主なパラメータ

| パラメータ名 | 型 | デフォルト値 | 説明 |
| --- | --- | --- | --- |
| `publish_rate` | double | `100.0` | `map -> odom` を publish する周期 [Hz]。 |
| `map_frame` | string | `"map"` | 親フレーム名です。 |
| `odom_frame` | string | `"odom"` | 子フレーム名です。 |
| `base_frame` | string | `"base_link"` | `odom -> base_link` lookup に使うフレーム名です。 |

## 動作概要

1. 起動時は `map -> odom = identity` を内部状態として持ちます。
2. `publish_rate` 周期で現在の `map -> odom` を TF として publish します。
3. `/initialpose` を受けると、現在の `odom -> base_link` を lookup します。
4. `/initialpose` の `map -> base_link` 目標姿勢と `odom -> base_link` 現在姿勢から `map -> odom` を計算します。
5. 以後は更新後の `map -> odom` を継続 publish します。

計算式は次の通りです。

```text
map->odom = map->base_link_target * inverse(odom->base_link_current)
```

## `/initialpose` の意味

このノードでは `/initialpose` を「`map` から見た現在の `base_link` をこの姿勢にしたい」という要求として解釈します。

ここで重要なのは、`/initialpose` を受けても `odom -> base_link` は直接書き換えないことです。`r1_dummy_odometry_node` が持っている `odom -> base_link` はそのままにして、`map -> odom` を更新することで最終的な `map -> base_link` を合わせます。

関係式は次の通りです。

```text
map->base_link = map->odom * odom->base_link
```

`/initialpose` で与えられるのは左辺の目標値 `map->base_link_target` です。現在の `odom->base_link_current` は TF から取得できるので、

```text
map->base_link_target = map->odom * odom->base_link_current
```

を満たす `map->odom` を解いています。

その結果、

```text
map->odom = map->base_link_target * inverse(odom->base_link_current)
```

となります。

### 具体例

- 現在の `odom -> base_link` が `(x=1.0, y=0.0, yaw=0.0)`
- `/initialpose` で `(x=5.0, y=2.0, yaw=0.0)` を送る

とすると、`map -> odom` は概ね `(x=4.0, y=2.0, yaw=0.0)` になります。

すると最終的な `map -> base_link` は

```text
(map -> odom) * (odom -> base_link) = (5.0, 2.0, 0.0)
```

となり、地図上ではロボットが `/initialpose` の位置に移動したように見えます。

### 使っていないもの

- `covariance` は使っていません
- `frame_id` は `map` を期待しています
- `odom -> base_link` がまだ出ていないと計算できないため、その場合は更新しません

## 起動例

単体起動:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run r1_control r1_dummy_map_node --ros-args -p publish_rate:=100.0
```

`/initialpose` を送る例:

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

## デバッグのヒント

- TF 確認: `ros2 run tf2_ros tf2_echo map odom`
- TF 確認: `ros2 run tf2_ros tf2_echo odom base_link`
- `/initialpose` を送っても更新されない場合は、先に `odom -> base_link` が出ているか確認してください。
