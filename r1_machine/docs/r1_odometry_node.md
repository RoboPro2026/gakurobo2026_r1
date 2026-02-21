# r1_odometry_node

`r1_odometry_node` はメインボードから受け取る設置型エンコーダの速度と IMU のヨー角を用いて、ロボットの 2 次元オドメトリを計算し `nav_msgs/msg/Odometry` を `/odometry` トピックで配信する ROS 2 ノードです。周期 10 ms のタイマで定期的に姿勢と速度を更新しており、`odom` → `base_link` の座標系を前提としています。IMU を利用しない計測構成にも対応できるよう、ヨー角の参照はパラメータで無効化できます。

## トピック

- **Subscribe**
  - `/odometry_encoder` (`r1_msgs/msg/OdometryEncoder`): X・Y 方向のエンコーダ積算値と角速度。`encoder_pos_x`, `encoder_pos_y` は積算角度 [rad]、`encoder_speed_x`, `encoder_speed_y` は角速度 [rad/s] が格納されます。
  - `/bno086/imu/data_raw` (`sensor_msgs/msg/Imu`): IMU からのヨー角と角速度。`use_imu` が `true` の場合に限り、オドメトリの向きと角速度の参照として使用します。
  - `/set_odometry` (`std_msgs/msg/Float64MultiArray`): 実行中にオドメトリの基準を設定するトピック。`data` の先頭 3 要素を `[x, y, yaw]`（いずれも `odom` 座標系、単位は m/m/rad）として解釈し、`x/y` は内部の位置をその値へ直接設定します。`yaw` は IMU の現在ヨー角に対するオフセット（`offset_yaw`）を更新し、以降の配信ヨー角が指定値になるよう補正します（`yaw = normalize(imu_yaw + offset_yaw)`）。
- **Publish**
  - `/odometry` (`nav_msgs/msg/Odometry`): 推定された位置・姿勢・速度。`header.frame_id = "odom"`、`child_frame_id = "base_link"` 固定です。

## 主なパラメータ

`declare_parameter` で定義されているデフォルト値と役割は次の通りです。ノードは `add_on_set_parameters_callback` を使用しているため、実行中に `ros2 param set` で更新できます。

| パラメータ名 | 型 | デフォルト値 | 説明 |
| --- | --- | --- | --- |
| `wheel_radius` | double | `0.025` | 設置ホイールの半径 [m]。エンコーダ角度から移動距離へ換算する際に使用します。 |
| `encoder_x_inverse` | bool | `false` | X 方向エンコーダの正負を反転するか。`true` で時計回り/反時計回りの定義が逆転します。 |
| `encoder_y_inverse` | bool | `false` | Y 方向エンコーダの正負を反転するか。`true` で時計回り/反時計回りの定義が逆転します。 |
| `use_imu` | bool | `true` | IMU をオドメトリのヨー角・角速度に使用するか。`false` の場合は IMU サブスクリプションの更新を待たずに計算し、ヨー角はオフセットのみで表現されます。 |

## 計算の流れ

1. `/odometry_encoder` から受け取った積算角度 (`encoder_pos_*`) と角速度 (`encoder_speed_*`) を内部状態に保存します。積算量はいずれも [rad]、角速度は [rad/s] を想定しています。
2. 速度換算は `vx = direction_x * wheel_radius * encoder_speed_x`、`vy = direction_y * wheel_radius * encoder_speed_y` で行います。`direction_*` は `encoder_*_inverse` パラメータから ±1 を設定し、配線や設置向きの違いを吸収します。
3. `/bno086/imu/data_raw` から得たクォータニオンを Roll-Pitch-Yaw に変換し、Yaw 成分（`imu_yaw`）と Z 軸の角速度（`angular_velocity.z`）のみを使用します。`use_imu = false` の場合は IMU を参照せず、Yaw は `offset_yaw` のみ（角速度は 0）として扱われます。
4. Yaw（`yaw = normalize(imu_yaw + offset_yaw)`）を用いて、速度を `odom` 座標系に回転します（`vx_world`, `vy_world`）。
5. 位置の更新は、積算エンコーダ角度の差分から各周期の並進量（`px = wheel_radius * (encoder_pos_x - prev_encoder_pos_x)`、`py = wheel_radius * (encoder_pos_y - prev_encoder_pos_y)`）を求め、Yaw で `odom` 座標系へ回転した量（`px_world`, `py_world`）を内部の `pos_x/pos_y` に加算して積算します。
6. `nav_msgs/msg/Odometry` には `pose.pose.position.x/y = pos_x/pos_y`、`pose.pose.orientation` は Yaw のみから作成したクォータニオンを格納します（`pose.pose.position.z = 0.0`）。
7. 速度は `twist.twist.linear.x/y` に `odom` 座標系の `vx_world/vy_world`、`twist.twist.angular.z` に IMU の `angular_velocity.z` を格納します（`use_imu = false` の場合は 0）。

## 起動と確認

- 単体起動例:

  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 run r1_machine r1_odometry_node
  ```

- 主なデバッグコマンド:
  - `ros2 topic echo /odometry` で現在の推定値を確認。
  - `ros2 topic pub --once /set_odometry std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0]}"` のようにして、オドメトリ原点（位置と向き）を設定。

`r1_odometry_node` は車体の移動量と IMU からの向き情報を組み合わせたシンプルな 2D オドメトリを提供し、上位ノードからの自己位置推定の初期値やフィードバックとして利用できます。
