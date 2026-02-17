# r1_mecanum_node

`r1_mecanum_node` はロボットの操作指令 (`geometry_msgs/msg/Twist`) を受け取り、メカナムホイール4輪それぞれの角速度指令値を `r1_msgs/msg/Mecanum` で出力するROS 2ノードです。ロボット本体座標系での並進速度と角速度を逆運動学で変換し、ギア比やホイールの回転方向も考慮した値を算出します。また、実際のホイール回転 (`r1_msgs/msg/Mecanum`) を購読して順運動学を行い、推定されたロボット速度 (`geometry_msgs/msg/Twist`) を配信します。

BNO086 IMU (`/bno086/imu/data_raw`) に対応し、Yaw角を用いたフィールド指向（field-oriented）制御が可能です。`use_imu = true` の場合はIMUのYawを `theta` として用いて座標変換を行い、`false` の場合は `theta = 0`（IMU未使用）として計算します。

## 座標系とホイール番号

ノード内部では下図の座標系とホイール番号を用います。

```
  //    FL(0) ---- FR(1)
  //      |          |
  //      |   ロボット  |
  //      |          |
  //    RL(2) ---- RR(3)

  // FL: Front Left   FR: Front Right
  // RL: Rear Left    RR: Rear Right
  // 全モータが正回転すると、x方向に進むように定義。
  // 下のアスキーアートのときをロボットが0度のときと定義。
  //
  //                 y
  //                 ^
  //                 |
  //                 |
  //  FL(0) ↖︎       |          ↙ FR(1)
  //        \------------------/
  //        |        |         |
  //        |        |         |
  //        |                  | 
  //        |        |         |
  //  ------|--------O---------|----->x
  //        |       ↺ w       |
  //        |                  | 
  //        |                  |
  //        |                  | 
  //        /------------------\
  //  RL(2) ↙                  ↖︎ RR(3)
```

`r1_msgs/msg/Mecanum` のフィールド順も同じ (FL, FR, RL, RR) です。

## トピック

- **Subscribe**
  - `/cmd_vel` (`geometry_msgs/msg/Twist`): ロボット基準座標系での速度指令 (m/s, rad/s)。
  - `/mecanum_wheel_speeds_feedback` (`r1_msgs/msg/Mecanum`): 実測ホイール角速度 [rad/s]。順運動学用。
  - `/bno086/imu/data_raw` (`sensor_msgs/msg/Imu`): IMUからの姿勢。`use_imu` が `true` のときYawを使用して座標変換します。
- **Publish**
  - `/mecanum_wheel_speeds_ref` (`r1_msgs/msg/Mecanum`): 各ホイールへの角速度指令 [rad/s]。フィールドは `fl/fr/rl/rr`。
  - `/mecanum_feedback_vel` (`geometry_msgs/msg/Twist`): `/mecanum_wheel_speeds_feedback` を順運動学で変換した推定速度。

## 主なパラメータ

`declare_parameter` で設定されているデフォルト値と役割は次のとおりです。すべての値は0より大きい必要があります (`motor_inverse` を除く)。

| パラメータ名 | 型 | デフォルト値 | 説明 |
| --- | --- | --- | --- |
| `wheel_radius` | double | `0.1` | ホイール半径 [m]。半径が小さいほど同じ速度でもモーター回転数が増えます。 |
| `robot_length` | double | `0.5` | ロボット前後方向のホイール間距離 [m]。 |
| `robot_width` | double | `0.25` | ロボット左右方向のホイール間距離 [m]。 |
| `speed_limit` | double | `100.0` | 各ホイール角速度の上限 [rad/s]。計算結果がこの値を超えると全輪同率でスケーリングされます。 |
| `gear_ratio` | double | `1.0` | 減速比。出力角速度は計算値を `gear_ratio` で割った後に配信されます。 |
| `use_imu` | bool | `true` | IMUを用いたYaw補正の有無。`true` で `/bno086/imu/data_raw` のYawを `theta` として用います。 |
| `motor_inverse` | bool[4] | `[false, false, false, false]` | 各ホイールの正回転方向を反転させるフラグ。`true` で-1倍されます。 |

ノードは `rclcpp::Node::add_on_set_parameters_callback` を用いているため、`ros2 param set` で実行中に値を変更できます (値が不正な場合は拒否されます)。

## 逆運動学の概要

受け取った速度指令 `(vx, vy, omega)` はロボットが角度 `theta` を持つ場合を想定し、回転行列でボディ座標へ変換してからメカナムホイールの逆運動学を適用しています。`use_imu = true` の場合は `theta` にIMUのYawを用い、`false` の場合は `theta = 0` とみなします。

```
vx_body =  vx_world * cos(theta) + vy_world * sin(theta)
vy_body = -vx_world * sin(theta) + vy_world * cos(theta)

// 逆運動学（実装に合わせた符号）
wheel_speeds[FL] = (1 / R) * (vx_body - vy_body - (L + W) * omega)
wheel_speeds[FR] = (1 / R) * (vx_body + vy_body - (L + W) * omega)
wheel_speeds[RL] = (1 / R) * (vx_body + vy_body + (L + W) * omega)
wheel_speeds[RR] = (1 / R) * (vx_body - vy_body + (L + W) * omega)
```

計算後にギア比・回転方向を反映し、`speed_limit` を超える場合は全体を `speed_limit / max(|wheel_speeds|)` でスケーリングします。

順運動学では `/mecanum_wheel_speeds_feedback` の値にモータ回転方向を再適用した後、以下の式を使ってボディ座標系速度を算出し、`theta` に基づく座標変換を行って `/mecanum_feedback_vel` として発行します。

```
vx = (R / 4) * (w_fl + w_fr + w_rl + w_rr)
vy = (R / 4) * (-w_fl + w_fr + w_rl - w_rr)
omega = (R / (4 * (L + W))) * (-w_fl - w_fr + w_rl + w_rr)
```

## 起動例

単体で起動する場合:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run r1_machine r1_mecanum_node
```

IMUを併用する場合は別途 `bno086` ノードを起動して `/bno086/imu/data_raw` を配信してください。
（例）

```bash
ros2 run bno086 bno086_node --ros-args -p port:="/dev/ttyACM0"
```

## パラメータ調整とデバッグ

- 動作中にパラメータを変更する例:  
  `ros2 param set /r1_mecanum_node speed_limit 20.0`
  `ros2 param set /r1_mecanum_node use_imu false`
- `r1_msgs/msg/Mecanum` 指令の確認:  
  `ros2 topic echo /mecanum_wheel_speeds_ref`
- フィードバック速度の確認:  
  `ros2 topic echo /mecanum_feedback_vel`
- `speed_limit` に達している場合は `fl/fr/rl/rr` が比例縮小されるため、モーター側でさらに制限を掛けたい場合は `sabacan` 側の `speed_lim` などと組み合わせて調整してください。

以上を踏まえて、`r1_mecanum_node` は `/cmd_vel` からの高レイヤ指令をメカナム駆動用のモーター角速度へ確実に変換する役割を担います。
