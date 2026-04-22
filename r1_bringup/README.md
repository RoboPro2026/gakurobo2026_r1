# r1_bringup

`r1_bringup` package に含まれる launch ファイルの概要です。

補足:

- 実機の通常起動は [`../scripts/r1_manual.bash`](../scripts/r1_manual.bash) または [`../scripts/r1_auto.bash`](../scripts/r1_auto.bash) を入口にすると整理しやすいです。
- 直接 `ros2 launch` を打つのは、`use_lidar` や `use_aruco_display` などの引数を切り替えたいときに限定する運用を推奨します。
- bag 記録は `ros2 bag record -a` ではなく、[`scripts/record.bash`](../scripts/record.bash) を使います。
- `record.bash` は CAN 関連 topic を除外して、その他の topic を記録します。

- [`r1_bringup.launch.py`](./docs/r1_bringup.launch.md)
  - 通常の実機起動とシミュレーション起動をまとめるメイン launch
- [`r1_slam.launch.py`](./docs/r1_slam.launch.md)
  - LiDAR と自己位置推定まわりを起動する launch
- [`lidar_lifecycle_watchdog_node`](./docs/lidar_lifecycle_watchdog_node.md)
  - 実機 LiDAR の lifecycle state を監視して `inactive` 取り残しを復帰するノード
- [`r1_sim.launch.py`](./docs/r1_sim.launch.md)
  - ダミー map / odometry を使うシミュレーション用 launch
- [`test_bringup.launch.py`](./docs/test_bringup.launch.md)
  - 足回りまわりのテスト用 launch
- [`test_slam_toolbox.launch.py`](./docs/test_slam_toolbox.launch.md)
  - `slam_toolbox` 単体検証用 launch
