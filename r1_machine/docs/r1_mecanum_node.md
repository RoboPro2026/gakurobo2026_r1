# r1_mecanum_node

`r1_mecanum_node` はロボットの操作指令 (`geometry_msgs/msg/Twist`) を受け取り、メカナムホイール4輪それぞれの角速度指令値を `std_msgs/msg/Float64MultiArray` で出力するROS 2ノードです。ロボット本体座標系での並進速度と角速度を逆運動学で変換し、ギア比やホイールの回転方向も考慮した値を算出します。

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
  // <---x--|--------O---------|
  //        |       ↺ w       |
  //        |                  | 
  //        |                  |
  //        |                  | 
  //        /------------------\
  //  RL(2) ↙                  ↖︎ RR(3)
```

`/mecanum_wheel_speeds` の配列インデックスも上記と同じ順序 (FL, FR, RL, RR) です。

## トピック

- **Subscribe**
  - `/cmd_vel` (`geometry_msgs/msg/Twist`): ロボット基準座標系での速度指令 (m/s, rad/s)。
- **Publish**
  - `/mecanum_wheel_speeds` (`std_msgs/msg/Float64MultiArray`): 各ホイールへの角速度指令 [rad/s]。配列の並びは `[FL, FR, RL, RR]`。

## 主なパラメータ

`declare_parameter` で設定されているデフォルト値と役割は次のとおりです。すべての値は0より大きい必要があります (`motor_inverse` を除く)。

| パラメータ名 | 型 | デフォルト値 | 説明 |
| --- | --- | --- | --- |
| `wheel_radius` | double | `0.1` | ホイール半径 [m]。半径が小さいほど同じ速度でもモーター回転数が増えます。 |
| `robot_length` | double | `0.5` | ロボット前後方向のホイール間距離 [m]。 |
| `robot_width` | double | `0.25` | ロボット左右方向のホイール間距離 [m]。 |
| `speed_limit` | double | `100.0` | 各ホイール角速度の上限 [rad/s]。計算結果がこの値を超えると全輪同率でスケーリングされます。 |
| `gear_ratio` | double | `1.0` | 減速比。出力角速度は計算値を `gear_ratio` で割った後に配信されます。 |
| `motor_inverse` | bool[4] | `[false, false, false, false]` | 各ホイールの正回転方向を反転させるフラグ。`true` で-1倍されます。 |

ノードは `rclcpp::Node::add_on_set_parameters_callback` を用いているため、`ros2 param set` で実行中に値を変更できます (値が不正な場合は拒否されます)。

## 逆運動学の概要

受け取った速度指令 `(vx, vy, omega)` はロボットが角度 `theta` (現在は0固定) を持つ場合を想定し、回転行列でボディ座標へ変換してからメカナムホイールの逆運動学を適用しています。

```
vx_body =  vx_world * cos(theta) + vy_world * sin(theta)
vy_body = -vx_world * sin(theta) + vy_world * cos(theta)

wheel_speeds[FL] = (1 / R) * (vx_body - vy_body + (L + W) * omega)
wheel_speeds[FR] = (1 / R) * (vx_body + vy_body + (L + W) * omega)
wheel_speeds[RL] = (1 / R) * (vx_body + vy_body - (L + W) * omega)
wheel_speeds[RR] = (1 / R) * (vx_body - vy_body - (L + W) * omega)
```

計算後にギア比・回転方向を反映し、`speed_limit` を超える場合は全体を `speed_limit / max(|wheel_speeds|)` でスケーリングします。

## 起動例

単体で起動する場合:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run r1_machine r1_mecanum_node
```

## パラメータ調整とデバッグ

- 動作中にパラメータを変更する例:  
  `ros2 param set /r1_mecanum_node speed_limit 20.0`
- 出力確認:  
  `ros2 topic echo /mecanum_wheel_speeds`
- `speed_limit` に達している場合は配列が比例縮小されるため、モーター側でさらに制限を掛けたい場合は `sabacan` 側の `speed_lim` などと組み合わせて調整してください。

以上を踏まえて、`r1_mecanum_node` は `/cmd_vel` からの高レイヤ指令をメカナム駆動用のモーター角速度へ確実に変換する役割を担います。
