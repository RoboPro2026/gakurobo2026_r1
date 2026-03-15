# r1_control
## 使用ライブラリ
- https://gitlab.com/libeigen/eigen
  - Eigen、連立方程式の計算に使用
- 台形加速
  - KERIさんのプログラムをベースとしている。ROS 2に適応させるため、一部改変
    - https://www.kerislab.jp/posts/2018-04-29-accel-designer1/
    - 自分で書いたら、どうやっても原作を超えられず、劣化版にしかならないので、原作を使用。まじですごい。
## 実装をするときに参考にしたもの
- https://github.com/surpace0924/simple_trajectory_generator_gui
- https://www.kerislab.jp/posts/2018-04-29-accel-designer4/

## ドキュメント

- [trajectory_planner](./docs/trajectory_planner.md)
- [r1_chassis_control_node](./docs/r1_chassis_control_node.md)
- [r1_dummy_odometry_node](./docs/r1_dummy_odometry_node.md)
- [r1_laser_filter_node](./docs/r1_laser_filter_node.md)
