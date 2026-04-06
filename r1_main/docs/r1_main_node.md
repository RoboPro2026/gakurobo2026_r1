# r1_main_node

`r1_main_node` は、R1 全体の高レベル制御を担当する ROS 2 ノードです。PS4 入力を受けて状態遷移を管理し、足回り・各機構・GPIO・自己位置初期化・`r1_machine_manage_node` への初期化要求をまとめて扱います。

現行実装では、機構 I/F を `register_position_axis()` / `register_velocity_axis()` / `register_gpio_*()` で登録し、各 helper 関数から publish する構成です。

## 現在の実装で重要な点

- 起動時の next state はパラメータ `robot_control_mode` で決まります。
  - 既定値は `manual`
  - `robot_control_mode:=manual` のとき `MANUAL / MODE1_DETECT_ORIGIN`
  - `robot_control_mode:=auto` のとき `AUTO / ACT0`
- `MainState` には `IDLE` / `EMERGENCY` / `MANUAL` / `AUTO` がありますが、現在のコードには main state を切り替える入力がありません。
- そのため通常運用では `MANUAL` しか入りません。`AUTO` を使う場合は `robot_control_mode:=auto` を指定して起動します。
- `PS` ボタンで `reset_robot(true)` と `/r1_machine_initialize` publish を行うまで、`is_initialized_ == false` のため各 mode の実動作は走りません。
- `MODE2_POLE` / `MODE3_SPEAR` / `MODE4_FKFS` / `MODE7_SPEAR_ATTACK` は、現状ほとんどの処理がコメントアウトされています。

## 役割

- `/joy` を受けて PS4 入力状態を更新する。
- 状態遷移を `StateMachine` で管理する。
- `cmd_vel_topic` で指定した topic に速度指令を publish して足回りへ送る。
- 各機構へ位置指令、速度指令、GPIO 指令、原点検出指令を publish する。
- `/set_mecanum_yaw`、`/set_swerve_drive_yaw`、`/set_odometry`、`/initialpose` を publish して姿勢・自己位置を初期化する。
- `PS` ボタン押下時に `/r1_machine_initialize` を publish して、`r1_machine_manage_node` 側の復帰処理を開始する。
- 自動回収中は `map -> base_link` TF を用いて KFS 回収範囲への進入判定を行う。

## 状態遷移

### MainState

- `IDLE`
- `EMERGENCY`
- `MANUAL`
- `AUTO`

### ManualSubState

- `MODE1_DETECT_ORIGIN`
- `MODE2_POLE`
- `MODE3_SPEAR`
- `MODE4_FKFS`
- `MODE5_RKFS`
- `MODE6_R2_LIFT`
- `MODE7_SPEAR_ATTACK`
- `TEST`

### AutoSubState

- `ACT0`

### 現在の実行上の挙動

- 起動時は `robot_control_mode` パラメータに応じた初期状態を使用
- `robot_control_mode:=manual` なら `MANUAL / MODE1_DETECT_ORIGIN`
- `robot_control_mode:=auto` なら `AUTO / ACT0`
- `share` ボタンで `MANUAL` 内の sub state を次の順で巡回
  - `MODE1_DETECT_ORIGIN`
  - `MODE2_POLE`
  - `MODE3_SPEAR`
  - `MODE4_FKFS`
  - `MODE5_RKFS`
  - `MODE6_R2_LIFT`
  - `MODE7_SPEAR_ATTACK`
  - `MODE1_DETECT_ORIGIN`
## 主なトピック

### Subscribe

- `/joy` (`sensor_msgs/msg/Joy`)
- `/bno086/imu/data_raw` (`sensor_msgs/msg/Imu`)
- `/odometry` (`nav_msgs/msg/Odometry`)
- `/chassis_act_status` (`std_msgs/msg/Int32`)
- `/<axis>_mode_status` (`std_msgs/msg/Int32`)
  - 対象軸
  - `kfs_fx`
  - `kfs_fz`
  - `kfs_fyaw`
  - `kfs_rx`
  - `kfs_rz`
  - `kfs_ryaw`
  - `spear1`
  - `spear2`
  - `spear3`
  - `spear4`
  - `spear_x`
  - `spear_y`
  - `spear_roll`
  - `spear_pitch1`
  - `spear_pitch2`
- `/<gpio>_status` (`r1_msgs/msg/GpioInput`)
  - `kfs_fz_low_switch`
  - `kfs_rz_low_switch`

### Publish

- `cmd_vel_topic` (`geometry_msgs/msg/Twist`)
- `/set_mecanum_yaw` (`std_msgs/msg/Float64`)
- `/set_swerve_drive_yaw` (`std_msgs/msg/Float64`)
- `/set_odometry` (`std_msgs/msg/Float64MultiArray`)
- `/initialpose` (`geometry_msgs/msg/PoseWithCovarianceStamped`)
- `/chassis_act_ref` (`std_msgs/msg/Int32`)
- `/robot_move` (`r1_msgs/msg/RobotMove`)
- `/r1_machine_initialize` (`std_msgs/msg/Empty`)
- `/<axis>_position_ref` (`std_msgs/msg/Float64`)
- `/<axis>_detect_origin` (`std_msgs/msg/Bool`)
- `/r2_flift_motor_ref` (`r1_msgs/msg/MotorRef`)
- `/r2_rlift_motor_ref` (`r1_msgs/msg/MotorRef`)
- `/<gpio>_gpio_pwm_ref` (`r1_msgs/msg/GpioPwmRef`)
  - `kfs_front_pump`
  - `kfs_rear_pump`
  - `kfs_front_valve`
  - `kfs_rear_valve`

### Sabacan 関連 Publish

- `/sabacan_power_ref0` (`sabacan_msgs/msg/SabacanPowerRef`)
- `/sabacan_led_ref1` (`sabacan_msgs/msg/SabacanLEDRef`)

## 登録している機構

### Position Axis

- `kfs_fx`
- `kfs_fz`
- `kfs_fyaw`
- `kfs_rx`
- `kfs_rz`
- `kfs_ryaw`
- `spear1`
- `spear2`
- `spear3`
- `spear4`
- `spear_x`
- `spear_y`
- `spear_roll`
- `spear_pitch1`
- `spear_pitch2`

### Velocity Axis

- `r2_flift`
- `r2_rlift`

### GPIO Output

- `kfs_front_pump`
- `kfs_rear_pump`
- `kfs_front_valve`
- `kfs_rear_valve`

### GPIO Input

- `kfs_fz_low_switch`
- `kfs_rz_low_switch`

## 主な helper 関数

- 足回り
  - `chassis_move_vel(vx, vy, omega)`
- KFS
  - `kfs_fx()`
  - `kfs_fz()`
  - `kfs_fyaw()`
  - `kfs_rx()`
  - `kfs_rz()`
  - `kfs_ryaw()`
- R2 昇降
  - `r2_flift()`
  - `r2_rlift()`
- やり
  - `spear1()`
  - `spear2()`
  - `spear3()`
  - `spear4()`
  - `spear_x()`
  - `spear_y()`
  - `spear_roll()`
  - `spear_pitch1()`
  - `spear_pitch2()`
- 原点検出
  - `*_detect_origin()`
- GPIO
  - `kfs_front_pump()`
  - `kfs_rear_pump()`
  - `kfs_front_valve()`
  - `kfs_rear_valve()`

## PS4 操作

### 共通操作

- 左スティック / 右スティック
  - `MANUAL` 中はそのまま `cmd_vel_topic` へ反映します。
  - `AUTO` 中も `chassis_act_status_ == NONE` の間は手動速度指令を送れます。
- `options`
  - `sabacan_power_ref(!sabacan_is_ems_)` を送り、電源基板の EMS をトグルします。
- `ps`
  - `reset_robot(true)` を実行します。
  - `/r1_machine_initialize` を publish します。
- `share`
  - `MANUAL` 中は manual sub state を順送りします。
  - `AUTO` 中は現状 `ACT0` のままで、実質何も変わりません。

### `MANUAL / MODE1_DETECT_ORIGIN`

現在有効なのは次の 2 つだけです。

- `circle`
  - `kfs_rz_detect_origin()`
- `cross`
  - `kfs_ryaw_detect_origin()`

### `MANUAL / MODE5_RKFS`

- `up`
  - `kfs_rz` を 1 段上の preset へ移動
  - `LOW -> MIDDLE -> HIGH -> BOOK`
- `down`
  - `kfs_rz` を 1 段下の preset へ移動
- `right`
  - `kfs_rx` を `NORMAL <-> EXPAND` でトグル
- `left`
  - `kfs_rear_pump` を ON/OFF
  - OFF 時は `kfs_rear_valve` を 250 ms だけ開けてから閉じます
- `triangle`
  - `kfs_ryaw` を `FRONT -> SIDE -> REAR` へ進める
- `cross`
  - `kfs_ryaw` を 1 段戻す意図の処理
  - ただし現実装では `kfs_fyaw(...)` を呼んでおり、意図通り動かない可能性があります
- `circle`
  - `kfs_ryaw` を `+0.1` rad 微調整
- `square`
  - `kfs_ryaw` を `-0.1` rad 微調整
- `l1` / `r1`
  - `kfs_rx` を `-0.01 / +0.01`
- `l2` / `r2`
  - `kfs_rz` を `-0.01 / +0.01`

### `MANUAL / MODE6_R2_LIFT`

- `triangle` を押している間
  - 前後の lift を上昇方向へ速度指令
- `cross` を押している間
  - 前後の lift を下降方向へ速度指令
- どちらも押していない間
  - 両方停止

### 現状ほぼ未実装の mode

- `MODE2_POLE`
- `MODE3_SPEAR`
- `MODE4_FKFS`
- `MODE7_SPEAR_ATTACK`

これらは関数自体は残っていますが、大半の操作がコメントアウトされています。

## `AUTO / ACT0`

`AUTO` は現在デフォルトで入らないため、主にコード読解用メモです。

### `chassis_act_status_ == NONE` のときの操作

- `triangle`
  - `publish_robot_move(ChassisAct::ACT0_START, {}, {})`
- `circle`
  - 青ゾーン用の開始姿勢を設定
  - `set_mecanum_yaw(0.0)`
  - `set_odometry(-5.5, 0.5, 0.0)`
  - `set_initialpose(-5.5, 0.5, 0.0)`
- `cross`
  - 内回り KFS 回収用の `RobotMove` を publish
- `square`
  - 外回り KFS 回収用の `RobotMove` を publish
- `down`
  - `publish_robot_move(ChassisAct::ACT3_START, {}, {})`

### 自動回収の挙動

- `ACT1` / `ACT2` 中は `map -> base_link` TF を見て、回収範囲の長方形に入ったかを判定します。
- 判定対象の中心座標は `inner_collect_kfs_center_pos.*` / `outer_collect_kfs_center_pos.*` です。
- `zone == blue` のときは `x` と `yaw` を反転して使用します。
- `collect_kfs_offset` を、使用する KFS 機構に応じて中心座標へ加えます。
- 範囲内なら LED を緑、範囲外なら赤にします。

### 制約

- `collect_kfs_type` の割り当ては実装上ほぼ青ゾーン前提です。
- 赤ゾーン側は TODO が残っており、未整備です。
- `sabacan_led_update()` は空実装です。

## `reset_robot(bool is_start_zone)`

`reset_robot(is_start_zone)` では次を行います。

- 各 step カウンタを初期化
- メンバー変数 `zone_` を `blue` / `red` として検証します
- `zone_ == blue` なら開始姿勢を `(-5.5, 0.5, 0.0)` に設定します
- `zone_ == red` なら開始姿勢を `(5.5, 0.5, 0.0)` に設定します
- `/set_mecanum_yaw` と `/set_swerve_drive_yaw` に開始 yaw を送信します
- `/set_odometry` と `/initialpose` に開始姿勢を送信します
- 速度制御系とポンプ・バルブを停止
- 初期 state に戻します
- `is_initialized_ = true`

`is_start_zone == false` の分岐は現状まだ TODO で、今は start zone と同じ開始姿勢を使います。

`/r1_machine_initialize` publish 自体は `reset_robot()` の外で行っており、`PS` ボタン押下時にセットで実行されます。

## パラメータ

実際の bringup 設定は [`r1_machine_config.yaml`](/home/user/ros2_ws/src/gakurobo2026_r1/r1_bringup/config/r1_machine_config.yaml) の `r1_main_node` セクションにあります。
bringup 起動時は [`r1_bringup.launch.py`](/home/user/ros2_ws/src/gakurobo2026_r1/r1_bringup/launch/r1_bringup.launch.py) の `robot_control_mode` 引数を `r1_main_node` に渡します。

### 基本

- `zone`
  - `blue` または `red`
- `cmd_vel_topic`
- `timer_rate`
- `ps4_connection_timeout`

### 足回り

- `chassis_max_velocity`
- `chassis_max_omega`

### KFS

- `kfs_fx_normal_pos`
- `kfs_fx_expand_pos`
- `kfs_fz_normal_pos`
- `kfs_fz_low_pos`
- `kfs_fz_middle_pos`
- `kfs_fz_high_pos`
- `kfs_fz_book_pos`
- `kfs_fyaw_normal_angle`
- `kfs_fyaw_front_angle`
- `kfs_fyaw_side_angle`
- `kfs_fyaw_rear_angle`
- `kfs_rx_normal_pos`
- `kfs_rx_expand_pos`
- `kfs_rz_normal_pos`
- `kfs_rz_low_pos`
- `kfs_rz_middle_pos`
- `kfs_rz_high_pos`
- `kfs_rz_book_pos`
- `kfs_ryaw_normal_angle`
- `kfs_ryaw_front_angle`
- `kfs_ryaw_side_angle`
- `kfs_ryaw_rear_angle`

### R2 昇降

- `r2_lift_up_velocity`
- `r2_lift_down_velocity`

### KFS 回収経路

- `kfs_forest_number`
- `inner_collect_kfs_center_pos.<1..12>`
- `outer_collect_kfs_center_pos.<1..12>`
- `collect_kfs_height`
- `collect_kfs_width`
- `collect_kfs_offset`

## 実装メモ

- `register_position_axis()` は `position_ref` publish、`detect_origin` publish、`mode_status` subscribe をまとめて登録します。
- `register_velocity_axis()` は `MotorRef` を `control_type = "VELOCITY"` で publish します。
- `publish_*` helper は publish と同時に内部の ref 値も更新します。
- `set_initialpose()` は既定で 0.2 秒遅延してから `/initialpose` を 1 回だけ publish します。
- `ps4_->is_connected() == false` の間は、`MANUAL` / `AUTO` とも危険側のアクチュエータを停止します。

## Launch

- 通常の bringup では [`r1_bringup.launch.py`](/home/user/ros2_ws/src/gakurobo2026_r1/r1_bringup/launch/r1_bringup.launch.py) から起動します。
- パラメータは [`r1_machine_config.yaml`](/home/user/ros2_ws/src/gakurobo2026_r1/r1_bringup/config/r1_machine_config.yaml) から読み込みます。
- bringup では `cmd_vel_topic` を `/cmd_vel_target` に設定し、[`r1_chassis_velocity_control_node`](/home/user/ros2_ws/src/gakurobo2026_r1/r1_control/docs/r1_chassis_velocity_control_node.md) を経由して最終的な `/cmd_vel` に変換されます。
- `robot_control_mode:=manual` なら `MANUAL / MODE1_DETECT_ORIGIN`、`robot_control_mode:=auto` なら `AUTO / ACT0` で起動します。

## 起動例

bringup から手動機で起動:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch r1_bringup r1_bringup.launch.py robot_control_mode:=manual
```

bringup から自動機で起動:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch r1_bringup r1_bringup.launch.py robot_control_mode:=auto
```

単体起動:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run r1_main r1_main_node --ros-args -p zone:=blue -p robot_control_mode:=manual
```

## デバッグ例

- 状態遷移ログ
  - `ros2 topic echo /rosout`
- 機構初期化信号
  - `ros2 topic echo /r1_machine_initialize`
- R2 昇降指令
  - `ros2 topic echo /r2_flift_motor_ref`
  - `ros2 topic echo /r2_rlift_motor_ref`
- KFS スイッチ
  - `ros2 topic echo /kfs_fz_low_switch_status`
  - `ros2 topic echo /kfs_rz_low_switch_status`
