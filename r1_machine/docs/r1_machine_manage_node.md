# r1_machine_manage_node

`r1_machine_manage_node` は、Sabacan 系トピックと `r1_msgs` 系トピックの橋渡しを行う ROS 2 ノードです。足回り、各機構、GPIO をまとめて扱い、Sabacan の生 status を論理的な topic に整理して再配信します。

現行実装では、足回りの制御方式を `drive_mode` パラメータで `mecanum` / `swerve` に切り替えられます。`mecanum` では従来の `/mecanum_wheel_speeds_ref` と `/mecanum_wheel_speeds_feedback` を使い、`swerve` では `/swerve_drive_ref` から 4 輪の wheel / steer 指令へ分解して Sabacan 単軸指令へ流します。

また、`/sabacan_power_status0` の EMS / SOFT EMS を監視し、非常停止中は `r1_machine_manage_node` 内で全モータ指令を open-loop 停止へ強制します。Robomas は `TORQUE 0.0`、VESC は `CURRENT 0.0` を出し、非常停止解除後も `/r1_machine_initialize` を受け取るまでは open-loop 停止を維持します。

## 役割

- Sabacan の `robomas_status` / `gpio_status` を購読し、機構ごとの状態 topic や debug topic に再配信する。
- `MotorRef` / `GpioPwmRef` / `GpioServoRef` を Sabacan 単軸制御 topic へ変換する。
- `mecanum` モードでは足回りの速度指令とオドメトリエンコーダを扱う。
- `swerve` モードでは `/swerve_drive_ref` を 4 輪の wheel / steer 指令へ分解する。
- `SabacanPowerStatus` による非常停止中は、全モータ指令を open-loop 停止へ上書きする。
- 非常停止解除後は `/r1_machine_initialize` 受信まで open-loop 停止を継続する。

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
- `r2_lift` の速度制御系チャネルはこのモードでも有効です。

補足:

- 現行実装では、KFS などの既存機構チャネルは主に `mecanum` 側構成として扱っています。`r2_lift` は両 mode で有効です。
- `swerve` モード時は mecanum のフィードバック publish とオドメトリエンコーダ publish は停止します。

## 主なトピック

### Common Subscribe

- `/sabacan_robomas_status0` ... `/sabacan_robomas_status9` (`sabacan_msgs/msg/SabacanRobomasStatus`)
- `/sabacan_gpio_status0` ... `/sabacan_gpio_status9` (`sabacan_msgs/msg/SabacanGPIOStatus`)
- `/sabacan_power_status0` (`sabacan_msgs/msg/SabacanPowerStatus`)
- `/r1_machine_initialize` (`std_msgs/msg/Empty`)

### Common Publish

- `/sabacan_gpio_ref_int0` ... `/sabacan_gpio_ref_int9` (`sabacan_msgs/msg/SabacanGPIORefInt`)
- `/sabacan_gpio_ref_float0` ... `/sabacan_gpio_ref_float9` (`sabacan_msgs/msg/SabacanGPIORefFloat`)

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
  - `/spear_rotate_angle_motion_status`
  - `/spear_hand_valve*_gpio_pwm_ref`
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
補足:

- `drive_mode` は実行中の `ros2 param set` で更新できます。
- timer 系パラメータは起動時に読み込まれます。
- 独ステの board_id / motor_number はソースコード内で固定しています。
  - wheel 側: `[1, 0]`, `[1, 1]`, `[1, 2]`, `[1, 3]`
  - steer 側: `[2, 0]`, `[2, 1]`, `[2, 2]`, `[2, 3]`
- 独ステの control_type もソースコード内で固定しています。
  - wheel 側: `VELOCITY`
  - steer 側: `POSITION`
- 非常停止中の open-loop 停止はパラメータではなく固定動作です。
  - Robomas: `TORQUE 0.0`
  - VESC: `CURRENT 0.0`

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
