# r1_slam.launch.py

実機の LiDAR と自己位置推定まわりを起動する launch です。`r1_bringup.launch.py` から include される前提で使います。

## 主な役割

- `urg_node2_1` と `urg_node2_2` を lifecycle node として起動する
- `base_link -> laser1` と `base_link -> laser2` の static TF を publish する
- `dual_laser_merger` で `/scan1` と `/scan2` を `/scan` に統合する
- `lidar_lifecycle_watchdog_node` で LiDAR lifecycle state を監視し、`inactive` 取り残しを復帰させる
- `nav2_map_server` と `nav2_amcl` で地図ベースの自己位置推定を動かす

## 引数

この launch に公開引数はありません。`r1_bringup.launch.py` の `use_lidar:=true` から include される想定です。

## 主に起動するノード

- `urg_node2_1` (`LifecycleNode`)
- `urg_node2_2` (`LifecycleNode`)
- `static_transform_publisher` (`base_link -> laser1`)
- `static_transform_publisher` (`base_link -> laser2`)
- `dual_laser_merger`
- `lidar_lifecycle_watchdog_node`
- `map_server`
- `amcl`
- `lifecycle_manager_localization`

## LiDAR lifecycle 復帰

`urg_node2_1` と `urg_node2_2` は launch event で `configure -> activate` へ自動遷移します。

起動タイミングによって片方だけ `inactive` に残る場合があるため、追加対策として以下を入れています。

- [`lidar_lifecycle_watchdog_node`](./lidar_lifecycle_watchdog_node.md) が周期的に lifecycle state を確認し、`inactive` なら `activate`、`unconfigured` なら `configure` を要求する
- `configuring` や `activating` などの遷移中 state では追加遷移を要求せず、起動時の通常遷移と競合しないようにする

これにより `/scan1` は出るが `/scan2` が出ず、結果として `/scan` や `map -> odom` が出ない状態からの自動復帰を狙います。

## 補足

- `slam_toolbox` と `r1_laser_filter_node` の起動コードはファイル内にありますが、現状はコメントアウトされています。
- 地図ファイルは `src/gakurobo2026_r1/data/map/field_blue.yaml` を参照しています。
- AMCL は `/scan` を購読します。`/scan1` と `/scan2` のどちらかが止まると `dual_laser_merger` が `/scan` を publish できず、AMCL の `map -> odom` TF も出ません。

## 起動例

```bash
ros2 launch r1_bringup r1_slam.launch.py
```
