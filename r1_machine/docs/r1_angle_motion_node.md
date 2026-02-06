# r1_angle_motion_node

`r1_angle_motion_node` は回転軸用 Robomas モータの位置制御と原点検出を行う ROS 2 ノードです。`/angle_motion_status` で取得したトルク・速度・角度・リミットスイッチを監視し、通常は角度指令を `/angle_motion_motor_ref` に出力します。原点検出要求が入ると速度モードへ切り替え、スイッチまたはトルク上昇を検出してオフセットを更新し、その場で位置モードに戻します。角度指令はラジアン単位で受け付け、減速比 `gear_ratio` でモータ角度へ換算します。

## トピック

- **Subscribe**
  - `/angle_motion_status` (`r1_msgs/msg/AngleMotion`): トルク [Nm], 速度 [rad/s], 角度 [rad] を取得。
- `/low_switch_status`(`r1_msgs/msg/GpioInput`): スイッチの値。 `inverse_*data` で XOR 反転されます。
- `/high_switch_status`(`r1_msgs/msg/GpioInput`): スイッチの値。 `inverse_*data` で XOR 反転されます。
  - `/angle_motion_position_ref` (`std_msgs/msg/Float64`): 目標角度 [rad]。原点検出中（速度モード）は無視されます。
  - `/angle_motion_detect_origin` (`std_msgs/msg/Bool`): `true` で原点検出モードへ移行し、`false` で位置モードに戻します。
- **Publish**
  - `/angle_motion_motor_ref` (`r1_msgs/msg/MotorRef`): Robomas へ渡す制御指令。`control_type` は `"POSITION"` または `"VELOCITY"`、`ref` は角度 [rad] もしくは角速度 [rad/s]。
  - `/angle_motion_mode_status` (`std_msgs/msg/Int32`): モードを送信。mode=0のとき、通常動作（位置制御モード）。mode=1のとき、原点復帰中（速度制御モード）。

## 主なパラメータ

`declare_parameter` で設定されているデフォルト値と役割は次のとおりです。`add_on_set_parameters_callback` を使っているため、`ros2 param set` で実行中に更新できます。

| パラメータ名 | 型 | デフォルト値 | 説明 |
| --- | --- | --- | --- |
| `use_low_switch` | bool | `true` | 低側リミットスイッチを原点検出判定に使うか。 |
| `use_high_switch` | bool | `true` | 高側リミットスイッチを原点検出判定に使うか。 |
| `torque_threshold` | double | `1.0` | 許容トルク [Nm]。この絶対値を超え続けたら原点とみなします。 |
| `origin_detect_threshold_time` | double | `0.1` | トルクしきい値超過を原点とみなすまでの継続時間 [s]。 |
| `origin_detect_speed` | double | `-3.14` | 原点検出中に流す一定角速度 [rad/s]。符号は `inverse_motor` に応じて反転します。 |
| `angle_min` | double | `-3.14` | 指令として受け付ける角度の下限 [rad]。外れた場合はクランプされます。 |
| `angle_max` | double | `3.14` | 指令として受け付ける角度の上限 [rad]。 |
| `gear_ratio` | double | `0.05` | 減速比。出力角度は計算値を `gear_ratio` で割った後に配信されます。 |
| `inverse_motor` | bool | `false` | モータ正転方向を反転するフラグ。`true` で `ref` に -1 を掛けます。 |
| `inverse_low_switch_logic` | bool | `false` | 低側リミットスイッチの論理を反転します。 |
| `inverse_high_switch_logic` | bool | `false` | 高側リミットスイッチの論理を反転します。 |

## 動作の流れ

1. `/angle_motion_status` を受信してトルク・速度・角度・リミットスイッチ状態を内部に保持します。
2. 通常は位置モード（`MODE_POSITION`）。`/angle_motion_position_ref` で受けた目標角度 [rad] を `angle_min`〜`angle_max` にクランプし、原点検出で決まる `angle_offset` を加算してから、`target_motor_angle = (target_angle) / gear_ratio` として `"POSITION"` 指令を配信します（`inverse_motor` で符号反転）。速度モード中はこのトピックを無視します。
3. `/angle_motion_detect_origin` に `true` を送ると速度モード（原点検出）へ移行し、10 ms 周期のタイマで `"VELOCITY"` 指令 `origin_detect_speed` を流し続けます。検出条件は以下の OR です。  
   - `use_low_switch`/`use_high_switch` が有効で、対応するスイッチがオン。  
   - `|torque| > torque_threshold` の状態が `origin_detect_threshold_time` 秒以上続く。
4. 検出条件を満たすと、現在の `pos` から `angle_offset = gear_ratio * pos` を設定し、その場の角度で `"POSITION"` 指令を出して位置モードへ復帰します。`/angle_motion_detect_origin` に `false` を送れば手動でも位置モードへ戻せます（オフセット更新は行いません）。

## 起動と利用例

- 単体起動例:

  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 run r1_machine r1_angle_motion_node
  ```

- 原点検出を開始する:

  ```bash
  ros2 topic pub /angle_motion_detect_origin std_msgs/Bool '{data: true}'
  ```

- 原点決め後に 1.0 rad へ移動させる指令:

  ```bash
  ros2 topic pub /angle_motion_position_ref std_msgs/Float64 '{data: 1.0}'
  ```

- 状態確認: `ros2 topic echo /angle_motion_status`。トルクやリミットスイッチがしきい値判定に正しく反映されているかをここで確認できます。
