# r1_main_node

`r1_main_node` は、R1 全体の高レベル制御を担当する ROS 2 ノードです。PS4 入力、状態遷移、足回り指令、各機構への目標値 publish、`r1_machine_manage_node` への初期化要求、自己位置初期化要求をまとめて扱います。

現行実装では、各機構の topic を `register_position_axis` / `register_velocity_axis` / `register_gpio_*` で登録し、`manual_task()` と `auto_task()` から共通 helper を通して publish する構成になっています。

## 役割

- `/joy` を受けて PS4 入力状態を更新する。
- 状態遷移を `StateMachine` で管理し、`IDLE` / `EMERGENCY` / `MANUAL` / `AUTO` を切り替える。
- 足回りへ `/cmd_vel` を publish する。
- 各機構へ位置指令、速度指令、GPIO 指令、原点検出指令を publish する。
- yaw / odometry / initialpose の初期化を行う。
- PS ボタン押下時に `/r1_machine_initialize` を publish して、`r1_machine_manage_node` 側で Sabacan reset と機構初期化を開始させる。

## 状態遷移

- main state:
  - `IDLE`
  - `EMERGENCY`
  - `MANUAL`
  - `AUTO`
- manual sub state:
  - `MODE1_DETECT_ORIGIN`
  - `MODE2_POLE`
  - `MODE3_SPEAR`
  - `MODE4_FKFS`
  - `MODE5_RKFS`
  - `MODE6_R2_LIFT`
  - `MODE7_SPEAR_ATTACK`
  - `TEST`
- auto sub state:
  - `ACT0`

現状の constructor では、起動時の next state は `AUTO / ACT0` に設定されています。

## 主なトピック

### Subscribe

- `/joy` (`sensor_msgs/msg/Joy`)
- `/bno086/imu/data_raw` (`sensor_msgs/msg/Imu`)
- `/odometry` (`nav_msgs/msg/Odometry`)
- `/chassis_act_status` (`std_msgs/msg/Int32`)
- `/<axis>_mode_status` (`std_msgs/msg/Int32`)
  - 対象軸:
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
  - 対象入力:
    - `kfs_fz_low_switch`
    - `kfs_rz_low_switch`

### Publish

- `/cmd_vel` (`geometry_msgs/msg/Twist`)
- `/set_mecanum_yaw` (`std_msgs/msg/Float64`)
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
  - 対象出力:
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

## 主な関数

- 足回り:
  - `chassis_move_vel(vx, vy, omega)`
- KFS:
  - `kfs_fx()`
  - `kfs_fz()`
  - `kfs_fyaw()`
  - `kfs_rx()`
  - `kfs_rz()`
  - `kfs_ryaw()`
- R2 昇降:
  - `r2_flift()`
  - `r2_rlift()`
- やり:
  - `spear1()`
  - `spear2()`
  - `spear3()`
  - `spear4()`
  - `spear_x()`
  - `spear_y()`
  - `spear_roll()`
  - `spear_pitch1()`
  - `spear_pitch2()`
- 原点検出:
  - `*_detect_origin()` の wrapper を各軸に用意
- GPIO:
  - `kfs_front_pump()`
  - `kfs_rear_pump()`
  - `kfs_front_valve()`
  - `kfs_rear_valve()`

## PS4 操作の要点

- `options`:
  - `sabacan_power_ref(!sabacan_is_ems_)` を送り、電源基板の EMS を切り替える。
- `ps`:
  - `reset_robot()` を実行する。
  - あわせて `/r1_machine_initialize` を publish する。
- `share`:
  - `MANUAL` 中は manual sub state を順送りする。
  - `AUTO` 中は現状 `ACT0` を維持する。

`reset_robot()` の内容は次の通りです。

- 各手順の step を初期化する。
- `/set_mecanum_yaw` に `0.0` を送る。
- `/set_odometry` に `(0.0, 0.0, 0.0)` を送る。
- 危険なアクチュエータを停止する。
- Sabacan reset と motion node の initialize は `/r1_machine_initialize` を受けた `r1_machine_manage_node` 側で実行する。

## パラメータ

`r1_main_node` では主に次のパラメータ群を使います。実際の bringup 設定は [`r1_machine_config.yaml`](/home/user/ros2_ws/src/gakurobo2026_r1/r1_bringup/config/r1_machine_config.yaml) の `r1_main_node` セクションにあります。

- 基本:
  - `zone`
  - `timer_rate`
- 足回り:
  - `chassis_max_velocity`
  - `chassis_max_omega`
- KFS 位置・角度:
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
- R2 昇降:
  - `r2_lift_max_velocity`
- KFS 回収経路:
  - `kfs_forest_number`
  - `inner_collect_kfs_center_pos.<1..12>`
  - `outer_collect_kfs_center_pos.<1..12>`
  - `collect_kfs_height`
  - `collect_kfs_width`
  - `collect_kfs_offset`

## 実装メモ

- `register_position_axis()` で登録した軸は、`position_ref` と `detect_origin` の publish と `mode_status` の購読をまとめて扱います。
- `register_velocity_axis()` で登録した軸は、`MotorRef(control_type="VELOCITY")` を publish します。
- `register_gpio_pwm_output()` / `register_gpio_input()` で GPIO をまとめています。
- `publish_*` helper は publish と同時に内部の ref 変数も更新します。
- ポール系や一部の旧 manual task は現在コメントアウトされており、refactor 途中のまま残っています。

## Launch

- 通常の bringup では [`r1_bringup.launch.py`](/home/user/ros2_ws/src/gakurobo2026_r1/r1_bringup/launch/r1_bringup.launch.py) から `r1_main_node` が起動されます。
- パラメータは [`r1_machine_config.yaml`](/home/user/ros2_ws/src/gakurobo2026_r1/r1_bringup/config/r1_machine_config.yaml) から読み込まれます。

## 起動例

```bash
source ~/ros2_ws/install/setup.bash
ros2 run r1_main r1_main_node --ros-args -p zone:=blue
```

## デバッグ例

- 現在の状態遷移ログを確認:
  - `ros2 topic echo /rosout`
- 機構初期化信号を確認:
  - `ros2 topic echo /r1_machine_initialize`
- R2 昇降指令を確認:
  - `ros2 topic echo /r2_flift_motor_ref`
  - `ros2 topic echo /r2_rlift_motor_ref`
- KFS スイッチを確認:
  - `ros2 topic echo /kfs_fz_low_switch_status`
  - `ros2 topic echo /kfs_rz_low_switch_status`
