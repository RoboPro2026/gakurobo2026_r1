# r1_bringup

`r1_bringup` package に含まれる launch ファイルの概要です。

- [`r1_bringup.launch.py`](/home/user/ros2_ws/src/gakurobo2026_r1/r1_bringup/docs/r1_bringup.launch.md)
  - 通常の実機起動とシミュレーション起動をまとめるメイン launch
- [`r1_slam.launch.py`](/home/user/ros2_ws/src/gakurobo2026_r1/r1_bringup/docs/r1_slam.launch.md)
  - LiDAR と自己位置推定まわりを起動する launch
- [`r1_sim.launch.py`](/home/user/ros2_ws/src/gakurobo2026_r1/r1_bringup/docs/r1_sim.launch.md)
  - ダミー map / odometry を使うシミュレーション用 launch
- [`test_bringup.launch.py`](/home/user/ros2_ws/src/gakurobo2026_r1/r1_bringup/docs/test_bringup.launch.md)
  - 足回りまわりのテスト用 launch
- [`test_slam_toolbox.launch.py`](/home/user/ros2_ws/src/gakurobo2026_r1/r1_bringup/docs/test_slam_toolbox.launch.md)
  - `slam_toolbox` 単体検証用 launch
