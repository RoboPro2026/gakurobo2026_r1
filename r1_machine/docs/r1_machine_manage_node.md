# r1_machine_manage_node

`r1_machine_manage_node` は、Sabacan 系トピックと `r1_msgs` 系トピックの橋渡しを行う ROS 2 ノードです。足回り、各機構、GPIO をまとめて扱い、Sabacan の生 status を論理的な topic に整理して再配信します。

現行実装では、足回りの制御方式を `drive_mode` パラメータで `mecanum` / `swerve` に切り替えられます。`mecanum` では従来の `/mecanum_wheel_speeds_ref` と `/mecanum_wheel_speeds_feedback` を使い、`swerve` では `/swerve_drive_ref` から 4 輪の wheel / steer 指令へ分解して Sabacan 単軸指令へ流します。

また、`/sabacan_power_status0` の EMS / SOFT EMS を監視し、非常停止中は `r1_machine_manage_node` 内で全モータ指令を open-loop 停止へ強制します。Robomas は `TORQUE 0.0`、VESC / Robstride は `CURRENT 0.0` を出し、非常停止解除後も `/r1_machine_initialize` を受け取るまでは open-loop 停止を維持します。`/r1_machine_initialize` を受けたときは、open-loop 中に一度 initialize を送り、その後 Sabacan reset を順番に発行し、reset 完了後にもう一度 initialize を送って所定時間待ってから通常制御へ戻します。

## Sabacan 構成

- Robomas は `board_id=1..7` を使用します。
- GPIO は `board_id=1..3` を使用します。
- Robstride は `board_id=1` を使用します。
- Power と LED はそれぞれ 1 台だけを単独で扱い、配列化していません。
- これらの使用数は `r1_machine_manage_node` 内の定数 `kSabacanRobomasNumber`、`kSabacanGpioNumber`、`kSabacanRobstrideNumber`、`kSabacanPowerNumber`、`kSabacanLedNumber` を変更すると追従します。
- `board_id` を index に使う内部 vector は、最大 board_id に合わせて `0..7` の slot を確保しています。
- reset service は `/sabacan_power_reset`、`/sabacan_robomas_reset_id1..7`、`/sabacan_gpio_reset_id1..3`、`/sabacan_led_reset` を使用します。

## 役割

- Sabacan の `robomas_status` / `robstride_status` / `gpio_status` を購読し、機構ごとの状態 topic や debug topic に再配信する。
- `MotorRef` / `GpioPwmRef` / `GpioServoRef` を Sabacan 単軸制御 topic へ変換する。
- 各 motion node から受けた torque limit 指令を、対応する Robomas 基板の `SetRobomasGains` service へ転送する。
- `sabacan_single_control_node` が単軸の `control_type` を切り替えるときも、同じ `SetRobomasGains` service を利用して Robomas 基板内で原子的に更新する。
- `mecanum` モードでは足回りの速度指令とオドメトリエンコーダを扱う。
- `swerve` モードでは `/swerve_drive_ref` を 4 輪の wheel / steer 指令へ分解する。
- 足回り以外の機構チャネルは drive mode に関係なく有効で、Robomas と Robstride の両方を扱える。
- `SabacanPowerStatus` による非常停止中は、全モータ指令を open-loop 停止へ上書きする。
- 非常停止解除後は `/r1_machine_initialize` 受信まで open-loop 停止を継続する。
- `/r1_machine_initialize` を受けると、open-loop 中に一度 initialize を送り、その後 Sabacan reset を順番に実行する。
- reset 完了後は linear / angle motion node へ initialize 信号を再度中継し、`swerve` モードでは `/swerve_drive_initialize` も再度 publish する。
- post-reset initialize の送信後は一定時間だけ open-loop 停止を維持し、その後に通常制御へ戻す。

## 初期化と非常停止

### 通常時の `/r1_machine_initialize`

- `/r1_machine_initialize` を受けると、`r1_machine_manage_node` は Sabacan reset シーケンスを開始します。
- reset シーケンス中は通常指令を通さず、全モータ指令を open-loop 停止へ強制します。
- reset 開始直前に、open-loop 停止のまま linear motion node、angle motion node、`swerve` モード用 initialize を先に 1 回 publish します。
- reset service は `power -> robomas -> gpio -> led` の順に呼び出します。
- reset 完了後、linear motion node と angle motion node へ initialize をもう一度中継します。
- `swerve` モードでは、そのとき `/swerve_drive_initialize` ももう一度 1 回 publish します。
- 2 回目の initialize 後は `post_reset_initialize_delay_sec` のあいだ open-loop 停止を維持し、その後に通常制御へ戻します。
- 非常停止由来ではない通常の initialize でも、Sabacan reset と motion node initialize は実行されます。

### 非常停止に入ったとき

- `/sabacan_power_status0` の EMS / SOFT EMS が active になると、`emergency_feedback_active_ = true` と `emergency_reinit_required_ = true` になります。
- この時点で全モータへ open-loop 停止を即時 publish します。
- 以後は `POSITION` / `VELOCITY` などの通常指令を受けても、そのまま Sabacan へは流さず、0 指令の open-loop 制御へ上書きします。
- open-loop 停止の内容は固定です。
  - Robomas: `TORQUE 0.0`
  - VESC / Robstride: `CURRENT 0.0`

### 非常停止を解除した直後

- EMS / SOFT EMS が inactive になると `emergency_feedback_active_` は false になります。
- ただし `emergency_reinit_required_` は残るため、この時点ではまだ通常制御には戻りません。
- 非常停止解除だけではモータ指令は復帰せず、全モータは open-loop 停止のままです。
- この状態では `/r1_machine_initialize` を受けるまで、位置制御や速度制御の指令はすべて open-loop 停止へ変換されます。

### 非常停止解除後に `/r1_machine_initialize` を受けたとき

- まだ EMS / SOFT EMS が active の場合、`/r1_machine_initialize` は無視されます。
- EMS / SOFT EMS が inactive なら、Sabacan reset シーケンスを開始します。
- reset 開始前に、open-loop 停止のまま initialize を 1 回送ります。
- reset シーケンス中も open-loop 停止は維持されます。
- reset 完了後に、linear motion node と angle motion node へ initialize をもう一度中継します。
- `swerve` モードでは、あわせて `/swerve_drive_initialize` をもう一度 publish し、`r1_swerve_drive_node` の steer 角キャッシュを現在の motor status に同期させます。
- その後、`post_reset_initialize_delay_sec` だけ待ってから `emergency_reinit_required_` を解除し、通常の motor routing を再開します。

### 状態遷移の要点

- 非常停止中: open-loop 停止
- 非常停止解除直後: まだ open-loop 停止
- `/r1_machine_initialize` 受信直後: pre-reset initialize を送るが、まだ open-loop 停止
- `/r1_machine_initialize` 受信後の reset 中: まだ open-loop 停止
- reset 完了直後: post-reset initialize を送るが、まだ open-loop 停止
- initialize 待ち時間満了後: 通常制御へ復帰

## Drive Mode

### `mecanum`

- `/mecanum_wheel_speeds_ref` を Sabacan の足回り 4 軸へ変換します。
- Sabacan の足回り status から `/mecanum_wheel_speeds_feedback` を生成します。
- Sabacan のエンコーダ status から `/odometry_encoder` を生成します。
- KFS、展開、ポール、やり、GPIO の既存機構もこのモードで有効です。

### `swerve`

- `/swerve_drive_ref` (`r1_msgs/msg/SwerveDrive`) を 4 輪の wheel / steer 指令へ分解して Sabacan へ流します。
- `/swerve_fr_wheel_motor_ref` などの単軸 `MotorRef` を直接受けて Sabacan へ流すこともできます。
- `/debug_swerve_*` 系の debug status を publish します。
- `r2_lift`、KFS、やりなどの機構チャネルもこのモードで有効です。

補足:

- `swerve` モード時は mecanum のフィードバック publish とオドメトリエンコーダ publish は停止します。
- `spear_roll` は Robstride (`/sabacan_robstride_ref2`, `/sabacan_robstride_status2`) 経由で扱います。
- `MotorRef.control_type = "POSITION"` を Robstride へ流すときは、Robstride の `PP` 位置指令へ変換します。

## 主なトピック

### Common Subscribe

- `/sabacan_robomas_status1` ... `/sabacan_robomas_status7` (`sabacan_msgs/msg/SabacanRobomasStatus`)
- `/sabacan_robstride_status1` (`sabacan_msgs/msg/SabacanRobstrideStatus`)
- `/sabacan_gpio_status1` ... `/sabacan_gpio_status3` (`sabacan_msgs/msg/SabacanGPIOStatus`)
- `/sabacan_power_status0` (`sabacan_msgs/msg/SabacanPowerStatus`)
- `/r1_machine_initialize` (`std_msgs/msg/Empty`)
- `*_torque_limit_ref` (`std_msgs/msg/Float64`): Robomas の linear / angle motion 軸だけを対象に購読し、対応 board / motor の `torque_lim` へ反映します。

### Common Publish

- `/sabacan_gpio_ref_int0` ... `/sabacan_gpio_ref_int7` (`sabacan_msgs/msg/SabacanGPIORefInt`)
- `/sabacan_gpio_ref_float0` ... `/sabacan_gpio_ref_float7` (`sabacan_msgs/msg/SabacanGPIORefFloat`)
- `/set_robomas_gains_id0` ... `/set_robomas_gains_id7` (`sabacan_msgs/srv/SetRobomasGains`)
  - `torque_lim` 更新に加えて、単軸 `control_type` 更新にも使用します。
- `/sabacan_power_reset` (`sabacan_msgs/srv/SabacanReset`)
- `/sabacan_robomas_reset_id1` ... `/sabacan_robomas_reset_id7`
- `/sabacan_gpio_reset_id1` ... `/sabacan_gpio_reset_id3`
- `/sabacan_led_reset`

### Mecanum Mode

- **Subscribe**
  - `/mecanum_wheel_speeds_ref` (`r1_msgs/msg/Mecanum`)
- **Publish**
  - `/mecanum_wheel_speeds_feedback` (`r1_msgs/msg/Mecanum`)
  - `/odometry_encoder` (`r1_msgs/msg/OdometryEncoder`)
  - `/debug_mecanum_fl_motor_status`
  - `/debug_mecanum_fr_motor_status`
  - `/debug_mecanum_rl_motor_status`
  - `/debug_mecanum_rr_motor_status`

### Swerve Mode

- **Subscribe**
  - `/swerve_drive_ref` (`r1_msgs/msg/SwerveDrive`)
  - `/swerve_fr_wheel_motor_ref` (`r1_msgs/msg/MotorRef`)
  - `/swerve_fl_wheel_motor_ref` (`r1_msgs/msg/MotorRef`)
  - `/swerve_rl_wheel_motor_ref` (`r1_msgs/msg/MotorRef`)
  - `/swerve_rr_wheel_motor_ref` (`r1_msgs/msg/MotorRef`)
  - `/swerve_fr_steer_motor_ref` (`r1_msgs/msg/MotorRef`)
  - `/swerve_fl_steer_motor_ref` (`r1_msgs/msg/MotorRef`)
  - `/swerve_rl_steer_motor_ref` (`r1_msgs/msg/MotorRef`)
  - `/swerve_rr_steer_motor_ref` (`r1_msgs/msg/MotorRef`)
- **Publish**
  - `/swerve_drive_initialize` (`std_msgs/msg/Empty`)
  - `/swerve_fr_wheel_motor_status` (`r1_msgs/msg/Motor`)
  - `/swerve_fl_wheel_motor_status` (`r1_msgs/msg/Motor`)
  - `/swerve_rl_wheel_motor_status` (`r1_msgs/msg/Motor`)
  - `/swerve_rr_wheel_motor_status` (`r1_msgs/msg/Motor`)
  - `/swerve_fr_steer_motor_status` (`r1_msgs/msg/Motor`)
  - `/swerve_fl_steer_motor_status` (`r1_msgs/msg/Motor`)
  - `/swerve_rl_steer_motor_status` (`r1_msgs/msg/Motor`)
  - `/swerve_rr_steer_motor_status` (`r1_msgs/msg/Motor`)
  - `/debug_swerve_fr_wheel_motor_status`
  - `/debug_swerve_fl_wheel_motor_status`
  - `/debug_swerve_rl_wheel_motor_status`
  - `/debug_swerve_rr_wheel_motor_status`
  - `/debug_swerve_fr_steer_motor_status`
  - `/debug_swerve_fl_steer_motor_status`
  - `/debug_swerve_rl_steer_motor_status`
  - `/debug_swerve_rr_steer_motor_status`

### 既存機構

以下の既存機構 topic は、主に `mecanum` モード側構成として処理されます。

- KFS:
  - `/kfs_fx_motor_ref`
  - `/kfs_fz_motor_ref`
  - `/kfs_fyaw_motor_ref`
  - `/kfs_rx_motor_ref`
  - `/kfs_rz_motor_ref`
  - `/kfs_ryaw_motor_ref`
  - `/kfs_*_linear_motion_status`
  - `/kfs_*_angle_motion_status`
- 展開:
  - `/front_expand_motor_ref`
  - `/rear_expand_motor_ref`
  - `/front_expand_linear_motion_status`
  - `/rear_expand_linear_motion_status`
- ポール:
  - `/pole_*_motor_ref`
  - `/pole_*_linear_motion_status`
  - `/pole_servo*_gpio_servo_ref`
  - `/pole_valve*_gpio_pwm_ref`
- やり:
  - `/spear_*_motor_ref`
  - `/spear_*_linear_motion_status`
  - `/spear_roll_angle_motion_status`
  - `/spear_pitch1_angle_motion_status`
  - `/spear_pitch2_angle_motion_status`
- GPIO:
  - `/kfs_front_pump_gpio_pwm_ref`
  - `/kfs_rear_pump_gpio_pwm_ref`
  - `/kfs_front_valve_gpio_pwm_ref`
  - `/kfs_rear_valve_gpio_pwm_ref`
  - `/brake_valve_gpio_pwm_ref`
  - `/kfs_front_switch_status`
  - `/kfs_rear_switch_status`
  - `/spear_move_switch_status`
  - `/spear_rotate_switch_status`

## パラメータ

| パラメータ名 | 型 | デフォルト値 | 説明 |
| --- | --- | --- | --- |
| `drive_mode` | string | `"mecanum"` | 足回りの制御方式。`mecanum` または `swerve`。 |
| `chassis_timer_rate` | double | `100.0` | シャーシ系フィードバック publish 周期 [Hz]。現状は `/mecanum_wheel_speeds_feedback` の publish に使います。 |
| `odometry_encoder_timer_rate` | double | `100.0` | オドメトリエンコーダ publish 周期 [Hz]。 |
| `linear_motion_timer_rate` | double | `100.0` | `LinearMotionChannel` 系 status 全体の publish 周期 [Hz]。 |
| `angle_motion_timer_rate` | double | `100.0` | `AngleMotionChannel` 系 status 全体の publish 周期 [Hz]。 |
| `velocity_control_timer_rate` | double | `100.0` | 速度制御系 status 全体の publish 周期 [Hz]。現状は `r2_lift` に使います。 |
| `gpio_timer_rate` | double | `100.0` | GPIO 入力状態 publish 周期 [Hz]。 |
| `sabacan_reset_send_interval_sec` | double | `1.0` | `sabacan_reset` service を順番に送る間隔 [s]。 |
| `post_reset_initialize_delay_sec` | double | `0.2` | reset 完了後に initialize を再送してから通常制御へ戻すまでの待ち時間 [s]。 |
補足:

- `drive_mode` は実行中の `ros2 param set` で更新できます。
- timer 系パラメータは起動時に読み込まれます。
- `sabacan_reset_send_interval_sec` と `post_reset_initialize_delay_sec` は実行中の `ros2 param set` でも更新できます。
- 独ステの board_id / motor_number はソースコード内で固定しています。
  - wheel 側: `[1, 0]`, `[1, 1]`, `[1, 2]`, `[1, 3]`
  - steer 側: `[2, 0]`, `[2, 1]`, `[2, 2]`, `[2, 3]`
- 足回りの controller_type もソースコード内で固定しています。
  - mecanum wheel: `VESC`
  - swerve wheel: `VESC`
  - swerve steer: `Robomas`
- 独ステの control_type もソースコード内で固定しています。
  - wheel 側: `VELOCITY`
  - steer 側: `POSITION`
- 非常停止中の open-loop 停止はパラメータではなく固定動作です。
  - Robomas: `TORQUE 0.0`
  - VESC / Robstride: `CURRENT 0.0`

## 設定ファイルと Launch

- bringup 用パラメータは [`r1_machine_config.yaml`](/home/user/ros2_ws/src/gakurobo2026_r1/r1_bringup/config/r1_machine_config.yaml) の `r1_machine_manage_node` セクションで設定します。
- 通常の bringup では [`r1_bringup.launch.py`](/home/user/ros2_ws/src/gakurobo2026_r1/r1_bringup/launch/r1_bringup.launch.py) から `r1_machine_manage_node` が起動されます。

## 起動例

```bash
source ~/ros2_ws/install/setup.bash
ros2 run r1_machine r1_machine_manage_node --ros-args -p drive_mode:=mecanum
```

独ステモードで起動する例:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run r1_machine r1_machine_manage_node --ros-args -p drive_mode:=swerve
```

## デバッグ例

- 現在の drive mode を確認:
  - `ros2 param get /r1_machine_manage_node drive_mode`
- メカナムフィードバックを確認:
  - `ros2 topic echo /mecanum_wheel_speeds_feedback`
- 独ステ debug status を確認:
  - `ros2 topic echo /debug_swerve_fr_wheel_motor_status`
  - `ros2 topic echo /debug_swerve_fr_steer_motor_status`
- 非常停止解除後に制御復帰させる:
  - `ros2 topic pub --once /r1_machine_initialize std_msgs/msg/Empty "{}"`
- この信号は非常停止ラッチ解除だけでなく、Sabacan reset と `r1_linear_motion_node` / `r1_angle_motion_node` の soft reset にも使われます。
- `swerve` モードでは、この復帰シーケンスの中で `/swerve_drive_initialize` も 1 回 publish されます。
