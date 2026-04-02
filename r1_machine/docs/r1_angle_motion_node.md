# r1_angle_motion_node

`r1_angle_motion_node` は回転軸用モータの位置制御、原点検出、機械端への押し当て移動を行う ROS 2 ノードです。`/angle_motion_status` で取得したトルク・速度・角度・リミットスイッチを監視し、通常は角度指令を `/angle_motion_motor_ref` に出力します。原点検出要求や `move_mech_lock` 要求が入ると速度モードへ切り替え、スイッチまたはトルク上昇を検出して停止します。原点検出ではオフセットを更新して通常角へ戻り、`move_mech_lock` ではオフセットを更新せず、その場の機械位置を保持します。`initialize` を受けた場合も、現在のモータ角度が論理上の 0 rad になるようにオフセットを更新してから、その場を保持します。加えて、通常時用と特殊動作用の 2 種類のトルク制限値を `/angle_motion_torque_limit_ref` へ publish し、`r1_machine_manage_node` 経由で Robomas 基板の `torque_lim` も切り替えます。角度指令はラジアン単位で受け付け、減速比 `gear_ratio` でモータ角度へ換算します。周期処理の実行レートは `timer_rate` で変更できます。

## トピック

- **Subscribe**
  - `/angle_motion_status` (`r1_msgs/msg/AngleMotion`): トルク [Nm], 速度 [rad/s], 角度 [rad] を取得。
- `/low_switch_status`(`r1_msgs/msg/GpioInput`): スイッチの値。 `inverse_*data` で XOR 反転されます。
- `/high_switch_status`(`r1_msgs/msg/GpioInput`): スイッチの値。 `inverse_*data` で XOR 反転されます。
  - `/angle_motion_position_ref` (`std_msgs/msg/Float64`): 目標角度 [rad]。原点検出中（速度モード）は無視されます。
  - `/angle_motion_detect_origin` (`std_msgs/msg/Bool`): `true` で原点検出モードへ移行し、`false` で位置モードに戻します。
  - `/angle_motion_move_mech_lock` (`std_msgs/msg/Int32`): 機械端まで押し当てる移動要求。`data > 0` で正方向、`data < 0` で逆方向、`data == 0` で停止して位置モードに戻ります。
  - `/angle_motion_initialize` (`std_msgs/msg/Empty`): 特殊モードを中断して位置モードへ戻し、その時点のモータ角度が論理上 0 rad になるよう `angle_offset` を更新してから保持します。`r1_machine_manage_node` から中継されます。
- **Publish**
  - `/angle_motion_motor_ref` (`r1_msgs/msg/MotorRef`): `r1_machine_manage_node` へ渡す制御指令。`control_type` は `"POSITION"` または `"VELOCITY"`、`ref` は角度 [rad] もしくは角速度 [rad/s]。
  - `/angle_motion_mode_status` (`std_msgs/msg/Int32`): モードを送信。mode=0のとき、通常動作（位置制御モード）。mode=1のとき、原点復帰中（速度制御モード）。
  - `/angle_motion_torque_limit_ref` (`std_msgs/msg/Float64`): 現在モードで使う Robomas の `torque_lim` [Nm]。通常時は `normal_torque_limit`、原点検出中と `move_mech_lock` 中は `contact_torque_limit` を出力します。

## 主なパラメータ

`declare_parameter` で設定されているデフォルト値と役割は次のとおりです。`add_on_set_parameters_callback` を使っているため、`ros2 param set` で実行中に更新できます。

| パラメータ名 | 型 | デフォルト値 | 説明 |
| --- | --- | --- | --- |
| `timer_rate` | double | `100.0` | 原点検出処理と `/angle_motion_mode_status` の周期更新レート [Hz]。 |
| `use_low_switch` | bool | `true` | 低側リミットスイッチを原点検出判定に使うか。 |
| `use_high_switch` | bool | `true` | 高側リミットスイッチを原点検出判定に使うか。 |
| `torque_threshold` | double | `1.0` | 許容トルク [Nm]。この絶対値を超え続けたら原点とみなします。 |
| `normal_torque_limit` | double | `1.0` | 通常位置モードで Robomas 基板へ設定する `torque_lim` [Nm]。 |
| `contact_torque_limit` | double | `1.0` | 原点検出中と `move_mech_lock` 中に Robomas 基板へ設定する `torque_lim` [Nm]。 |
| `origin_detect_threshold_time` | double | `0.2` | トルクしきい値超過を原点とみなすまでの継続時間 [s]。 |
| `origin_detect_speed` | double | `-3.14` | 原点検出中に流す一定角速度 [rad/s]。符号は `inverse_motor` に応じて反転します。 |
| `move_mech_lock_speed` | double | `3.14` | `move_mech_lock` 中に流す角速度の大きさ [rad/s]。向きは topic の符号で指定します。 |
| `angle_min` | double | `-3.14` | 指令として受け付ける角度の下限 [rad]。外れた場合はクランプされます。 |
| `angle_max` | double | `3.14` | 指令として受け付ける角度の上限 [rad]。 |
| `normal_angle` | double | `0.0` | 原点検出後の通常時の角度。 [rad]。 |
| `gear_ratio` | double | `0.05` | 減速比。出力角度は計算値を `gear_ratio` で割った後に配信されます。 |
| `inverse_motor` | bool | `false` | モータ正転方向を反転するフラグ。`true` で `ref` に -1 を掛けます。 |
| `inverse_low_switch_logic` | bool | `false` | 低側リミットスイッチの論理を反転します。 |
| `inverse_high_switch_logic` | bool | `false` | 高側リミットスイッチの論理を反転します。 |

## 動作の流れ

1. `/angle_motion_status` を受信してトルク・速度・角度・リミットスイッチ状態を内部に保持します。
2. 通常は位置モード（`MODE_POSITION`）。`/angle_motion_position_ref` で受けた目標角度 [rad] を `angle_min`〜`angle_max` にクランプし、原点検出で決まる `angle_offset` を加算してから、`target_motor_angle = (target_angle) / gear_ratio` として `"POSITION"` 指令を配信します（`inverse_motor` で符号反転）。速度モード中はこのトピックを無視します。
3. `/angle_motion_detect_origin` に `true` を送ると速度モード（原点検出）へ移行し、`timer_rate` 周期のタイマで `"VELOCITY"` 指令 `origin_detect_speed` を流し続けます。この切替と同時に `/angle_motion_torque_limit_ref` へ `contact_torque_limit` を publish します。検出条件は以下の OR です。  
   - `use_low_switch`/`use_high_switch` が有効で、対応するスイッチがオン。  
   - `|torque| > torque_threshold` の状態が `origin_detect_threshold_time` 秒以上続く。
4. 検出条件を満たすと、現在の `pos` から `angle_offset = gear_ratio * pos` を設定し、その場の角度で `"POSITION"` 指令を出して位置モードへ復帰します。`/angle_motion_detect_origin` に `false` を送れば手動でも位置モードへ戻せます（オフセット更新は行いません）。
5. `/angle_motion_move_mech_lock` に `1` または `-1` を送ると、指定方向へ `"VELOCITY"` 指令 `move_mech_lock_speed` を流し続けます。開始時に `/angle_motion_torque_limit_ref` へ `contact_torque_limit` を publish します。停止判定は原点検出と同じで、トルク上昇またはスイッチ反応が `origin_detect_threshold_time` 以上続いたときです。停止後はオフセットを更新せず、その時点のモータ位置を `"POSITION"` 指令で保持し、トルク制限も `normal_torque_limit` に戻します。
6. `/angle_motion_initialize` を受けると、原点検出中や `move_mech_lock` 中であっても速度モードを中断し、`angle_offset = gear_ratio * pos` を再計算してその時点のモータ角度が論理上 0 rad になるようにします。その後、現在のモータ角度を `"POSITION"` で保持します。このときトルク制限も `normal_torque_limit` に戻します。

## 起動と利用例

- 単体起動例:

  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 run r1_machine r1_angle_motion_node --ros-args -p timer_rate:=100.0
  ```

- 原点検出を開始する:

  ```bash
  ros2 topic pub /angle_motion_detect_origin std_msgs/Bool '{data: true}'
  ```

- 正方向へ機械端まで移動する:

  ```bash
  ros2 topic pub /angle_motion_move_mech_lock std_msgs/Int32 '{data: 1}'
  ```

- 原点決め後に 1.0 rad へ移動させる指令:

  ```bash
  ros2 topic pub /angle_motion_position_ref std_msgs/Float64 '{data: 1.0}'
  ```

- 状態確認: `ros2 topic echo /angle_motion_status`。トルクやリミットスイッチがしきい値判定に正しく反映されているかをここで確認できます。
