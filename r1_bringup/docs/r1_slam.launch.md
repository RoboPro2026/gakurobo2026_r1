# r1_slam.launch.py

実機の LiDAR と自己位置推定まわりを起動する launch です。`r1_bringup.launch.py` から include される前提で使います。

## 主な役割

- `urg_node2` を lifecycle node として起動する
- `base_link -> laser` の static TF を publish する
- `laser_filters/scan_to_scan_filter_chain` で `/scan` をフィルタする
- `nav2_map_server` と `nav2_amcl` で地図ベースの自己位置推定を動かす

## 引数

- `auto_start`
  - `true` のとき `urg_node2` を configure -> activate まで自動遷移させる
- `node_name`
  - `urg_node2` の node 名
- `scan_topic_name`
  - LiDAR の scan topic 名

## 主に起動するノード

- `urg_node2_node` (`LifecycleNode`)
- `static_transform_publisher`
- `scan_filter_chain`
- `map_server`
- `amcl`
- `lifecycle_manager_localization`

## 補足

- `slam_toolbox` と `r1_laser_filter_node` の起動コードはファイル内にありますが、現状はコメントアウトされています。
- 地図ファイルは `src/gakurobo2026_r1/data/map/field_blue.yaml` を参照しています。

## 起動例

```bash
ros2 launch r1_bringup r1_slam.launch.py
ros2 launch r1_bringup r1_slam.launch.py auto_start:=false
```
