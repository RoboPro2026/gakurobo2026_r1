# r1_swerve_drive_node

`r1_swerve_drive_node` は、速度指令 `/cmd_vel` (`geometry_msgs/msg/Twist`) を 4 輪独立ステアリング（swerve）用の指令 `/swerve_drive_ref` (`r1_msgs/msg/SwerveDrive`) に変換する ROS 2 ノードです。IMU (`/bno086/imu/data_raw`) のYaw（+ `yaw_offset`）を用いて、ロボットの向きを考慮した計算もできます（`use_imu`）。

手動で `/manual_swerve_drive_ref` を与えると、その値を（回転方向・ギア比・ステアオフセットを反映して）そのまま `/swerve_drive_ref` に流します。

## トピック

- **Subscribe**
  - `/cmd_vel` (`geometry_msgs/msg/Twist`): 速度指令（`linear.x`, `linear.y`, `angular.z`）。
  - `/manual_swerve_drive_ref` (`r1_msgs/msg/SwerveDrive`): 手動指令（limitチェックは行いません）。
  - `/bno086/imu/data_raw` (`sensor_msgs/msg/Imu`): IMU姿勢（`use_imu=true` のときYawを使用）。
  - `yaw_offset` (`std_msgs/msg/Float64`): Yawオフセット [rad]（`use_imu=true` のときのみ反映）。
- **Publish**
  - `/swerve_drive_ref` (`r1_msgs/msg/SwerveDrive`): 各輪の速度指令 `v0..v3` とステア角指令 `theta0..theta3`。

## パラメータ

`declare_parameter` で設定されているデフォルト値と役割は次のとおりです。`add_on_set_parameters_callback` を使っているため、`ros2 param set` で実行中に更新できます。

| パラメータ名 | 型 | デフォルト値 | 説明 |
| --- | --- | --- | --- |
| `robot_length` | double | `0.5` | ロボット長さ [m]。`robot_radius` の算出に使用。 |
| `robot_width` | double | `0.5` | ロボット幅 [m]。`robot_radius` の算出に使用。 |
| `wheel_radius` | double | `0.1` | ホイール半径 [m]（※現状の実装では計算に未使用）。 |
| `wheel_gear_ratio` | double | `1.0` | ホイール側の減速比（出力 `v*` は `... / wheel_gear_ratio`）。 |
| `steer_gear_ratio` | double | `1.0` | ステア側の減速比（出力 `theta*` は `... / steer_gear_ratio`）。 |
| `wheel_speed_limit` | double | `100.0` | 速度上限（※現状は超過時にログを出すのみで、指令値のスケーリングは反映されません）。 |
| `steer_angle_limit` | double | `6.28` | ステア角上限 [rad]（超過時にERRORログ）。 |
| `angle_diff_range` | double | `0.5` | ステア角連続化を行う角度差の範囲 [rad]（前回値との差がこの値未満のときに unwrap します）。 |
| `steer_theta_offset` | double[4] | `[0, 0, 0, 0]` | 各輪のステア角オフセット [rad]（`theta*` 出力に加算）。 |
| `wheel_motor_inverse` | bool[4] | `[false, false, false, false]` | 各輪の回転方向反転フラグ（`true` で `v*` に -1 倍を反映）。 |
| `steer_motor_inverse` | bool[4] | `[false, false, false, false]` | 各輪のステア回転方向反転フラグ（`true` で `theta*` に -1 倍を反映）。 |
| `use_imu` | bool | `true` | IMUのYawを計算に用いるか。`false` の場合はYawが更新されません（Yawを 0 として扱いたい場合は実装側で初期化が必要です）。 |

補足: `v0..v3` は内部で `vx_ref, vy_ref` から合成しており、現状の実装では `wheel_radius` による角速度換算は行っていません（`/cmd_vel` の並進速度と同じ次元の値になります）。

## 逆運動学（実装概要）

ノードは `robot_length` と `robot_width` から `robot_radius = sqrt((L/2)^2 + (W/2)^2)` を計算し、各輪が半径 `robot_radius` の円周上に 90 度間隔で配置されている近似を用いています。

入力 `(vx_ref, vy_ref, omega_ref)` と Yaw `theta` から、各輪 `i = 0..3` の速度ベクトルを次で計算します（実装の式）:

```
wheel_vx[i] = vx_ref - R * omega_ref * sin(theta + i*pi/2)
wheel_vy[i] = vy_ref + R * omega_ref * cos(theta + i*pi/2)
wheel_v[i]  = sqrt(wheel_vx[i]^2 + wheel_vy[i]^2)
steer_theta[i] = atan2(wheel_vy[i], wheel_vx[i])
```

ステア角 `steer_theta[i]` は前回指令 `prev_steer_theta_[i]` と比較し、角度差が `angle_diff_range` 未満のときに `-pi..pi` をまたがないように unwrap して連続性を保つようにしています。

最後に、回転方向・ギア比・ステアオフセットを反映して `/swerve_drive_ref` を出力します。

## 起動と利用例

単体起動例:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run r1_machine r1_swerve_drive_node
```

速度指令を与える例:

```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.5, y: 0.0}, angular: {z: 0.0}}'
```

手動で指令を与える例（limitチェックは行いません）:

```bash
ros2 topic pub /manual_swerve_drive_ref r1_msgs/msg/SwerveDrive '{v0: 1.0, v1: 1.0, v2: 1.0, v3: 1.0, theta0: 0.0, theta1: 0.0, theta2: 0.0, theta3: 0.0}'
```

指令値の確認:

```bash
ros2 topic echo /swerve_drive_ref
```
