# r1_odometry_node

`r1_odometry_node` はメインボードから受け取る設置型エンコーダの速度と IMU のヨー角を用いて、ロボットの 2 次元オドメトリを計算し `nav_msgs/msg/Odometry` を `/odometry` トピックで配信する ROS 2 ノードです。周期 10 ms のタイマで定期的に姿勢と速度を更新しており、`odom` → `base_link` の座標系を前提としています。IMU を利用しない計測構成にも対応できるよう、ヨー角の参照はパラメータで無効化できます。

## トピック

- **Subscribe**
  - `/odometry_encoder` (`r1_msgs/msg/OdometryEncoder`): X・Y 方向のエンコーダ積算値と角速度。`encoder_pos_x`, `encoder_pos_y` は積算角度 [rad]、`encoder_speed_x`, `encoder_speed_y` は角速度 [rad/s] が格納されます。
  - `/bno086/imu/data_raw` (`sensor_msgs/msg/Imu`): IMU からのヨー角と角速度。`use_imu` が `true` の場合に限り、オドメトリの向きと角速度の参照として使用します。
  - `/odometry_offset` (`std_msgs/msg/Float64MultiArray`): 実行中にオフセットを加算するためのトピック。`data` の先頭 3 要素を `[offset_pos_x, offset_pos_y, offset_yaw]` として解釈し、それぞれ現在値に加算します。
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
4. Yaw（`yaw = imu_yaw + offset_yaw`）を用いて、速度を `odom` 座標系に回転します（`vx_world`, `vy_world`）。その後 `dt = 0.01`（10 ms 固定）として `pos_x = vx_world * dt`、`pos_y = vy_world * dt` を計算します。
5. 位置は `offset_pos_x`, `offset_pos_y` を加算して `nav_msgs/msg/Odometry` に反映し、姿勢は Yaw のみから作成したクォータニオンを `pose.pose.orientation` に格納します（`pose.pose.position.z = 0.0`）。
6. 速度は `twist.twist.linear.x/y` にボディ座標系の `vx/vy` を、`twist.twist.angular.z` に IMU の `angular_velocity.z` を格納します（`use_imu = false` の場合は 0）。

注意: 現在の実装では `pos_x/pos_y` を内部で積算しておらず、各周期の変位（`vx_world * dt`, `vy_world * dt`）にオフセットを加えた値を `pose.pose.position.x/y` に格納します。

パラメータ更新は差し替えではなく加算で反映されるため、実行中に補正量を増減させたい場合は加法的に指定してください。

## 起動と確認

- 単体起動例:

  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 run r1_machine r1_odometry_node
  ```

- 主なデバッグコマンド:
  - `ros2 topic echo /odometry` で現在の推定値を確認。
  - `ros2 param set /r1_odometry_node offset_pos_x 0.05` のようにして実行中に位置補正を加算。
  - `ros2 topic pub --once /odometry_imu_offset std_msgs/msg/Float64MultiArray "{data: [0.05, 0.0, 0.0]}"` のようにして、トピック経由でオフセットを加算。

`r1_odometry_node` は車体の移動量と IMU からの向き情報を組み合わせたシンプルな 2D オドメトリを提供し、上位ノードからの自己位置推定の初期値やフィードバックとして利用できます。
