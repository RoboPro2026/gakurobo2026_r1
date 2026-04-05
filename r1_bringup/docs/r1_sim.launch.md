# r1_sim.launch.py

シミュレーション用の map / TF / odometry を立ち上げる launch です。`r1_bringup.launch.py` の `use_sim:=true` から利用します。

## 主な役割

- `nav2_map_server` で静的地図を publish する
- `r1_dummy_map_node` で `map -> odom` TF を publish する
- `r1_dummy_odometry_node` で `/odometry` と `odom -> base_link` TF を publish する

## 主に起動するノード

- `map_server`
- `lifecycle_manager_map_server`
- `r1_dummy_map_node`
- `r1_dummy_odometry_node`

## 参照する設定

- パラメータ: `config/r1_sim_config.yaml`
- 地図: `src/gakurobo2026_r1/data/map/field_blue.yaml`

## 起動例

```bash
ros2 launch r1_bringup r1_sim.launch.py
ros2 launch r1_bringup r1_bringup.launch.py use_sim:=true
```
