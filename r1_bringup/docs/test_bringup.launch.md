# test_bringup.launch.py

足回りと CAN 系の疎通確認に使うテスト用 launch です。`r1_main_node` ではなく `test_node` を起動して、Joy 入力からテスト用の指令を流します。

## 主な役割

- `joy_node` と `test_node` を起動する
- `bno086_node`、`r1_swerve_drive_node`、`eth2can_node` を起動する
- Sabacan 基板ノード群を少し遅らせて起動する
- 足回り 8 軸分の `sabacan_single_control_node` をさらに遅らせて起動する

## 引数

- `param_file`
  - 既定値は `config/test_config.yaml`
  - テスト用パラメータファイルを差し替えるときに使う

## 主に起動するノード

- `joy_node`
- `bno086_node`
- `r1_swerve_drive_node`
- `test_node`
- `eth2can_node`
- `sabacan_power_node_id0`
- `sabacan_robomasv2_node_id1`
- `sabacan_robomasv2_node_id2`
- `sabacan_single_control_node_id{1..2}_motor{0..3}`

## 起動順

- 即時起動
  - Joy、IMU、test_node、足回り制御、eth2can
- 1 秒後
  - Sabacan 基板ノード
- 2 秒後
  - single control ノード群

## 起動例

```bash
ros2 launch r1_bringup test_bringup.launch.py
ros2 launch r1_bringup test_bringup.launch.py param_file:=/tmp/test_config.yaml
```
