# lidar_lifecycle_watchdog_node

`lidar_lifecycle_watchdog_node` は、実機 LiDAR 用 `urg_node2` lifecycle node が起動時に `inactive` のまま残った場合に復帰させる監視ノードです。

## 主な役割

- 指定した lifecycle node の状態を周期的に確認する
- `unconfigured` の場合は `configure` を要求する
- `inactive` の場合は `activate` を要求する
- `active` の場合は何もしない

## 背景

`r1_slam.launch.py` では `urg_node2_1` と `urg_node2_2` を lifecycle node として起動し、`configure -> activate` の自動遷移を launch event で送っています。

起動タイミングによって片方の LiDAR が `inactive` に残ると、例えば `/scan1` は出るが `/scan2` が出ない状態になります。この場合 `dual_laser_merger` が `/scan` を publish できず、AMCL も `map -> odom` TF を出せません。

このノードは、そのような `inactive` 取り残しを検出して `activate` を再要求します。

## パラメータ

| パラメータ | 型 | デフォルト | 説明 |
| --- | --- | --- | --- |
| `node_names` | string array | `["urg_node2_1", "urg_node2_2"]` | 監視対象の lifecycle node 名です。先頭 `/` は省略できます。 |
| `check_period` | double | `1.0` | 状態確認周期 [s] です。 |
| `service_timeout` | double | `0.2` | lifecycle service 応答待ちタイムアウト [s] です。 |
| `retry_interval` | double | `2.0` | 同じ node に復帰遷移を再要求する最短間隔 [s] です。 |
| `configure_unconfigured` | bool | `true` | `unconfigured` を検出したときに `configure` を要求するかを指定します。 |
| `activate_inactive` | bool | `true` | `inactive` を検出したときに `activate` を要求するかを指定します。 |

## 起動

通常は `r1_slam.launch.py` から自動起動されます。

単体で起動する例:

```bash
ros2 run r1_bringup lidar_lifecycle_watchdog_node
```

監視対象を明示する例:

```bash
ros2 run r1_bringup lidar_lifecycle_watchdog_node --ros-args \
  -p node_names:="[urg_node2_1, urg_node2_2]"
```

## 確認

LiDAR が復帰しているかは次で確認します。

```bash
ros2 lifecycle get /urg_node2_1
ros2 lifecycle get /urg_node2_2
ros2 topic hz /scan1
ros2 topic hz /scan2
ros2 topic hz /scan
```

`/scan1` と `/scan2` が出ていれば、`dual_laser_merger` が `/scan` を publish できます。

## 注意

- このノードは lifecycle state の復帰要求だけを行います。LiDAR の電源断、USB断線、serial port 不一致など、service 自体が存在しない場合は復帰できません。
- `activate` が成功しても `/scan2` が出ない場合は、`urg_node2` のログ、デバイスパス、ケーブル、電源を確認してください。
