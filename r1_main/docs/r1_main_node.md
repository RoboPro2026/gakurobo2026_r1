# r1_main_node

`r1_main_node` は、R1 全体の高レベル制御を担当する ROS 2 ノードです。PS4 入力を受けて状態遷移を管理し、足回り・各機構・GPIO・自己位置初期化・`r1_machine_manage_node` への初期化要求をまとめて扱います。

現行実装では、機構 I/F を `register_position_axis()` / `register_velocity_axis()` / `register_gpio_*()` で登録し、各 helper 関数から publish する構成です。

## 現在の実装で重要な点

- 起動時の初期状態はパラメータ `robot_control_mode` で決まります。
  - 既定値は `manual`
  - `robot_control_mode:=manual` のとき `MainState=READY`、`OperationMode=MODE1_DETECT_ORIGIN`、`ChassisControlMode=MANUAL`
  - `robot_control_mode:=auto` のとき `MainState=READY`、`OperationMode=MODE1_DETECT_ORIGIN`、`ChassisControlMode=AUTO`
- 状態は `MainState` / `OperationMode` / `ChassisControlMode` の 3 軸で管理します。
- `MainState` はライフサイクルと安全状態、`OperationMode` は操作モード、`ChassisControlMode` は足回りの制御権を表します。
- これとは別に、シャーシ自動シーケンスは内部状態 `auto_chassis_status_`、KFS 自動回収は `kfs_auto_collect_status` で独立管理します。
- `PS` ボタンで `reset_robot(true)` と `/r1_machine_initialize` publish を行った後は、`/r1_machine_initialize_done` を受け取るまで `is_initialized_ == false` のため各ページの実動作は走りません。
- この待機中は、`PS` 押下後に右スティックで自動 ACT を再開することもできません。`r1_machine_initialize_done` を受け取ってから再度入力を受け付けます。
- `MODE2_POLE` / `MODE7_SPEAR_ATTACK` は、現状ほとんどの処理がコメントアウトされています。
- LED 指令は timer 周期ごとに `sabacan_led_update()` で 1 回だけ publish します。各処理は直接 publish せず、LED の要求状態を更新します。

## 役割

- `/joy` を受けて PS4 入力状態を更新する。
- 状態遷移を `StateMachine` で管理する。
- `cmd_vel_topic` で指定した topic に速度指令を publish して足回りへ送る。
- 各機構へ位置指令、速度指令、GPIO 指令、原点検出指令を publish する。
- `/set_mecanum_yaw`、`/set_swerve_drive_yaw`、`/set_odometry`、`/initialpose` を publish して姿勢・自己位置を初期化する。
- `PS` ボタン押下時に `/r1_machine_initialize` を publish して、`r1_machine_manage_node` 側の復帰処理を開始する。
- `PS` 初期化の開始時に、内部の auto chassis 状態と KFS 自動回収状態を破棄し、`r1_chassis_control_node` 側も `/r1_machine_initialize` を受けて追従内部状態を明示的にリセットする。
- KFS 自動回収中は `map -> base_link` TF を用いて KFS 回収範囲への進入判定を行う。
- KFS 自動回収の有効/無効は内部状態 `kfs_auto_collect_status` で管理し、`chassis_act_status_` とは独立に動作する。
- シャーシ自動開始要求は `map -> base_link` TF がまだ無い場合に即時 publish せず、自己位置推定が立ち上がるまで待機します。

## 状態遷移

### MainState

- `IDLE`
- `READY`
- `EMERGENCY`

### OperationMode

- `MODE1_DETECT_ORIGIN`
- `MODE2_POLE`
- `MODE3_SPEAR`
- `MODE4_FKFS`
- `MODE5_RKFS`
- `MODE6_R2_LIFT`
- `MODE7_SPEAR_ATTACK`
- `MODE8_AUTO_COLLECT_KFS`
- `MODE9_AUTO_CHASSIS`

### AutoChassisStatus

- `ChassisAct` 相当の内部状態を保持します。
- `NONE`
- `ACT0_START` / `ACT0` / `ACT0_FINISH`
- `ACT1_START` / `ACT1` / `ACT1_FINISH`
- `ACT2_START` / `ACT2` / `ACT2_FINISH`
- `ACT3_START` / `ACT3` / `ACT3_FINISH`
- `ACT4_START` / `ACT4` / `ACT4_FINISH`
- `ACT5_START` / `ACT5` / `ACT5_FINISH`
- `ACT_PAUSE` (1000) — 軌道追従を一時停止中（ポーズ状態）
- `ACT_RESUME` (1001) — ポーズ解除指令（遷移トリガー。定常状態ではない）

### ChassisControlMode

- `HOLD`
- `MANUAL`
- `AUTO`

### 現在の実行上の挙動

- 起動時は `robot_control_mode` パラメータに応じた初期状態を使用
- `robot_control_mode:=manual` なら `OperationMode=MODE1_DETECT_ORIGIN`
- `robot_control_mode:=auto` でも `OperationMode=MODE1_DETECT_ORIGIN`
- `share` ボタンで `OperationMode` を次の順で巡回
  - `MODE1_DETECT_ORIGIN`
  - `MODE2_POLE`
  - `MODE3_SPEAR`
  - `MODE4_FKFS`
  - `MODE5_RKFS`
  - `MODE6_R2_LIFT`
  - `MODE7_SPEAR_ATTACK`
  - `MODE8_AUTO_COLLECT_KFS`
  - `MODE9_AUTO_CHASSIS`
  - `MODE1_DETECT_ORIGIN`
- `ChassisControlMode` は `chassis_act_status_` に応じて自動更新されます。
  - `chassis_act_status_ == NONE` または `ACT_PAUSE` のとき `MANUAL`
  - それ以外のとき `AUTO`
  - PS4 未接続時は `HOLD`
## 主なトピック

### Subscribe

- `/joy` (`sensor_msgs/msg/Joy`)
- `/bno086/imu/data_raw` (`sensor_msgs/msg/Imu`)
- `/odometry` (`nav_msgs/msg/Odometry`)
- `/chassis_act_status` (`std_msgs/msg/Int32`)
- `/r1_init_parameter` (`r1_msgs/msg/R1InitParameter`)
  - スマートフォンアプリ等から送られる初期化パラメータ。`zone`・`r1_kfs_value`・`enable_auto_select` を含みます。
  - MODE3 → 次モードへの右スティック遷移時に KFS 経路決定に使います。
- `/r1_collect_kfs` (`r1_msgs/msg/R1CollectKfs`)
  - 手動で KFS 回収経路を直接指定するときに使います。
  - `enable_auto_select == false` のときのみ、MODE3 遷移時の経路として採用されます。
- `/<axis>_mode_status` (`std_msgs/msg/Int32`)
  - 対象軸
  - `kfs_fx`
  - `kfs_fz`
  - `kfs_fyaw`
  - `kfs_rx`
  - `kfs_rz`
  - `kfs_ryaw`
  - `r2_flift`
  - `r2_rlift`
  - `spear_y`
  - `spear_roll1`
  - `spear_roll2`
- `/r1_kfs_mechanism_ref` (`std_msgs/msg/Int32`)
  - KFS 機構の目標状態を外部から直接指定するトピックです。値は `R1KfsMechanismRef` enum にキャストして使用します。
  - 有効な値: `NONE(-1)`, `FKFS_RACK(0)`, `FKFS_HIGH(1)`, `FKFS_MIDDLE(2)`, `FKFS_LOW(3)`, `FKFS_GROUND(4)`, `FKFS_STORAGE(5)`, `FKFS_COLLECT_START_POS(6)`, `RKFS_RACK(10)`, `RKFS_HIGH(11)`, `RKFS_MIDDLE(12)`, `RKFS_LOW(13)`, `RKFS_GROUND(14)`, `RKFS_STORAGE(15)`, `RKFS_COLLECT_START_POS(16)`
  - 上記以外の値を受信した場合は `RCLCPP_ERROR` を出力してその値を無視します。
- `/r1_retry_collect` (`std_msgs/msg/Int32`)
  - KFS 回収のリトライ要求。現在は受信ログのみ出力（未実装）。
- `/r1_collect_3rd_kfs` (`std_msgs/msg/Int32`)
  - 3 本目 KFS の回収番号を指定します。受信値を `r1_collect_3rd_kfs_` に保持します。
- `/r1_initialize_all_actuator` (`std_msgs/msg/Int32`)
  - 全アクチュエータ初期化要求。現在は受信ログのみ出力（未実装）。
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
- `/r1_machine_initialize_done` (`std_msgs/msg/Empty`) を subscribe
- `/chassis_tangent_pid_enable` (`std_msgs/msg/Bool`): KFS 回収ゾーン在圏中は `false` を publish して接線方向 PID を無効化します
- `/<axis>_position_ref` (`std_msgs/msg/Float64`)
- `/<axis>_detect_origin` (`std_msgs/msg/Bool`)
- `/aruco_marker_id` (`std_msgs/msg/Int32`): ArUco マーカー ID を publish します
- `/r1_operation_mode` (`std_msgs/msg/Int32`): 現在の `OperationMode` を publish します
- `/r1_log_message` (`std_msgs/msg/String`): ログメッセージを publish します
- `/<gpio>_gpio_pwm_ref` (`r1_msgs/msg/GpioPwmRef`)
  - `kfs_front_pump`
  - `kfs_rear_pump`
  - `kfs_front_valve`
  - `kfs_rear_valve`
  - `spear_hand1_valve`
  - `spear_hand2_valve`
  - `spear_hand_push_valve`

### Sabacan 関連 Publish

- `/sabacan_power_ref0` (`sabacan_msgs/msg/SabacanPowerRef`)
- `/sabacan_led_ref1` (`sabacan_msgs/msg/SabacanLEDRef`)

## LED の挙動

LED は timer callback の最後に 1 回だけ更新されます。

- `base` 表示
  - 現在の `next_state` から決まる通常表示です。
  - `READY` 中は `OperationMode` ごとに色が決まります。
  - `EMERGENCY` では赤点滅です。
- `status` 表示
  - その周期だけ有効な状態表示です。
  - 現在は KFS 回収範囲判定に使っており、範囲内なら緑、範囲外なら赤です。
- `event` 表示
  - 一定時間だけ `base` / `status` より優先されます。
  - 現在は `reset_robot(true)` 実行後に 1 秒間の青点滅を出します。

優先順位は `event > status > base` です。

点滅は `r1_main_node` 側で timer 位相から ON/OFF を計算して実現しています。周期指定は秒単位の `blink_period_s` を使い、`SabacanLEDRef` 自体には点滅情報は持たせていません。

### `OperationMode` の base 色

- `MODE1_DETECT_ORIGIN`
  - 青
- `MODE2_POLE`
  - 緑
- `MODE3_SPEAR`
  - シアン
- `MODE4_FKFS`
  - 赤
- `MODE5_RKFS`
  - マゼンタ
- `MODE6_R2_LIFT`
  - 黄
- `MODE7_SPEAR_ATTACK`
  - 白
- `MODE8_AUTO_COLLECT_KFS`
  - オレンジ
- `MODE9_AUTO_CHASSIS`
  - 黄

## 登録している機構

### Position Axis

- `kfs_fx`
- `kfs_fz`
- `kfs_fyaw`
- `kfs_rx`
- `kfs_rz`
- `kfs_ryaw`
- `r2_flift`
- `r2_rlift`
- `spear_y`
- `spear_roll1`
- `spear_roll2`

### GPIO Output

- `kfs_front_pump`
- `kfs_rear_pump`
- `kfs_front_valve`
- `kfs_rear_valve`
- `spear_hand1_valve`
- `spear_hand2_valve`
- `spear_hand_push_valve`

### GPIO Input

- `kfs_fz_low_switch`
- `kfs_rz_low_switch`

## 主な helper 関数

- 足回り
  - `chassis_move_vel(vx, vy, omega)`
- KFS
  - `kfs_fx_pos_ref()`
  - `kfs_fz_pos_ref()`
  - `kfs_fyaw_pos_ref()`
  - `kfs_rx_pos_ref()`
  - `kfs_rz_pos_ref()`
  - `kfs_ryaw_pos_ref()`
- R2 昇降
  - `r2_flift_pos_ref(pos)` / `r2_flift_set_pos(pos)`
  - `r2_flift_speed_ref(speed)` / `r2_flift_speed_mode_stop()`
  - `r2_rlift_pos_ref(pos)` / `r2_rlift_set_pos(pos)`
  - `r2_rlift_speed_ref(speed)` / `r2_rlift_speed_mode_stop()`
- やり
  - `spear_y_pos_ref(pos)` / `spear_y_set_pos(pos)`
  - `spear_y_speed_ref(speed)` / `spear_y_speed_mode_stop()`
  - `spear_roll1_pos_ref(angle)` / `spear_roll1_set_angle(angle)`
  - `spear_roll1_speed_ref(speed)` / `spear_roll1_speed_mode_stop()`
  - `spear_roll2_pos_ref(angle)` / `spear_roll2_set_angle(angle)`
  - `spear_roll2_speed_ref(speed)` / `spear_roll2_speed_mode_stop()`
- 原点検出
  - `*_detect_origin()`
- GPIO
  - `kfs_front_pump()`
  - `kfs_rear_pump()`
  - `kfs_front_valve()`
  - `kfs_rear_valve()`
  - `spear_hand1_valve(on)`
  - `spear_hand2_valve(on)`
  - `spear_hand_push_valve(on)`

## PS4 操作

### 共通操作

- 左スティック / 右スティック（傾け操作）
  - `ChassisControlMode == MANUAL` のとき `cmd_vel_topic` へ反映します。
  - つまり軌道実行中でない限り、`OperationMode` に関係なく手動速度指令を送れます。
- 右スティック（押し込み）
  - **ポーズ中**（`is_act_paused_ == true`）のとき: ポーズを解除してレジューム (`ACT_RESUME`) を publish し、軌道追従を再開します。
  - **自動走行中** (`chassis_act_status_ != NONE` かつ `!= ACT_PAUSE`) のとき:
    - `enable_right_stick_pause == true` かつ `ACT2 / ACT3 / ACT4 / ACT5` 実行中: `ACT_PAUSE` を publish して一時停止します。一時停止中は手動操作が可能です。
    - 上記以外（`enable_right_stick_pause == false`、または `ACT0 / ACT1` 等）: 自動走行を中断して手動モードへ戻ります（従来の挙動）。
  - **アイドル中**（自動走行していない）かつ `is_initialized_ == true` のとき: `OperationMode` に応じた ACT を開始します。
- `options`
  - `sabacan_power_ref(!sabacan_is_ems_)` を送り、電源基板の EMS をトグルします。
- `ps`
  - `activate_lidar_on_ps == true` のとき、`urg_node2_1` / `urg_node2_2` へ lifecycle `activate` を要求します。
  - `reset_robot(true)` を実行します（`received_r1_collect_kfs_` もリセットされます）。
  - `/r1_machine_initialize` を publish します。
  - `/initialpose` を計 3 回 publish します（`delay_sec` 後・`initialpose_retry1_delay_sec` 後・`initialpose_retry2_delay_sec` 後）。LiDAR の lifecycle activate 完了タイミングに依存せず AMCL に届けるためです。PS を連続で押した場合は前回の再送タイマーをすべてキャンセルしてから再スケジュールします。
- `share`
  - 短押し（`share_long_press_sec` 秒未満でリリース）: `OperationMode` を順送りします。
  - 長押し（`share_long_press_sec` 秒以上押し続けた時点で即時発火）: 一つ前の `OperationMode` へ戻ります。

### `OperationMode / MODE1_DETECT_ORIGIN`

現在有効なのは次の 2 つだけです。

- `circle`
  - `kfs_rz_detect_origin()`
- `cross`
  - `kfs_ryaw_detect_origin()`

### `OperationMode / MODE5_RKFS`

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
  - ただし現実装では `kfs_fyaw_pos_ref(...)` を呼んでおり、意図通り動かない可能性があります
- `circle`
  - `kfs_ryaw` を `+0.1` rad 微調整
- `square`
  - `kfs_ryaw` を `-0.1` rad 微調整
- `l1` / `r1`
  - `kfs_rx` を `-0.01 / +0.01`
  - `use_kfs_mech_lock == true` のとき、次の目標値が `kfs_rx_low_mech_lock_pos` / `kfs_rx_high_mech_lock_pos` を超える場合は通常の位置指令ではなく `move_mech_lock` を送ります
- `l2` / `r2`
  - `kfs_rz` を `-0.01 / +0.01`
  - `use_kfs_mech_lock == true` のとき、次の目標値が `kfs_rz_low_mech_lock_pos` / `kfs_rz_high_mech_lock_pos` を超える場合は通常の位置指令ではなく `move_mech_lock` を送ります

### `OperationMode / MODE4_FKFS`

- `up`
  - `kfs_fz` を 1 段上の preset へ移動
  - `LOW -> MIDDLE -> HIGH -> PUT`
- `down`
  - `kfs_fz` を 1 段下の preset へ移動
- `right`
  - `kfs_front_pump` を ON/OFF
  - OFF 時は `kfs_front_valve` を 250 ms だけ開けてから閉じます
- `triangle`
  - `kfs_fyaw` を `REAR -> SIDE -> FRONT` へ進める
- `circle`
  - `kfs_fx` を `NORMAL -> STORAGE -> PUT -> EXPAND` へ進める
- `cross`
  - `kfs_fyaw` を 1 段戻す
- `square`
  - `kfs_fx` を 1 段戻す
- `l1` / `r1`
  - `kfs_fx` を `-0.01 / +0.01`
  - `use_kfs_mech_lock == true` のとき、次の目標値が `kfs_fx_low_mech_lock_pos` / `kfs_fx_high_mech_lock_pos` を超える場合は通常の位置指令ではなく `move_mech_lock` を送ります
- `l2` / `r2`
  - `kfs_fz` を `-0.01 / +0.01`
  - `use_kfs_mech_lock == true` のとき、次の目標値が `kfs_fz_low_mech_lock_pos` / `kfs_fz_high_mech_lock_pos` を超える場合は通常の位置指令ではなく `move_mech_lock` を送ります

### `OperationMode / MODE8_AUTO_COLLECT_KFS`

- `triangle`
  - 内回り KFS 自動回収を開始
- `cross`
  - 外回り KFS 自動回収を開始
- `square`
  - KFS 自動回収を停止
- `circle`
  - 開始位置リセット
  - 併せて KFS 自動回収も停止
- `up` / `down`
  - `spear_x` を `+0.01 / -0.01`
- `left`
  - 前後 KFS のポンプを停止
  - バルブを 250 ms だけ開けてから閉じる
- `l1` / `r1`
  - `kfs_fx` を `-0.01 / +0.01`
- `l2` / `r2`
  - `kfs_fz` を `-0.01 / +0.01`

この mode では手動操縦を続けながら、KFS 自動回収だけを並行で動かせます。

### `OperationMode / MODE6_R2_LIFT`

- `triangle` を押している間
  - 前後の lift を上昇方向へ速度指令
- `cross` を押している間
  - 前後の lift を下降方向へ速度指令
- どちらも押していない間
  - 両方停止

### `OperationMode / MODE3_SPEAR`

右スティック押し込みで次モード（MODE4）へ遷移し、KFS 自動回収と自律シャーシ走行を開始します。

#### 経路決定ロジック

遷移時に `/r1_init_parameter` が未受信の場合は警告を出して遷移しません。

**`enable_auto_select == true` のとき（自動判別）**

`r1_kfs_value`（森番号 3 つ）を使って経路を決定します。

1. 3 つの森番号で INNER 森（1, 2, 4, 7, 10）と OUTER 森（3, 6, 9, 10, 11, 12）を多数決。
2. `INNER_ACTIVE` なら INNER 側の森のみ、`OUTER_ACTIVE` なら OUTER 側の森のみを抽出（最大 2 つ）。
3. 機構タイプの割り当て: INNER は先頭 `rear_kfs`・残り `front_kfs`、OUTER は先頭 `front_kfs`・残り `rear_kfs`。

無効な森番号（範囲外・5・8）が含まれる場合は警告を出してその値を読み飛ばし、残りの有効な値で経路を決定します。有効な値が 1 つも無い場合は遷移しません。

**`enable_auto_select == false` のとき（手動指定）**

`/r1_collect_kfs` の `forest_order` と `kfs_mechanism_type` をそのまま使います。

- `/r1_collect_kfs` が未受信・空の場合は警告を出して遷移しません。
- 各森番号のバリデーション（1〜12、5 と 8 は除く）を行い、無効値があれば遷移しません。
- `forest_order` の多数決で `INNER_ACTIVE` / `OUTER_ACTIVE` を決定します（`auto_collect_kfs_task` の座標選択に必要）。

#### シャーシ ACT の選択

| 状態 | 条件 | 使用 ACT |
|---|---|---|
| OUTER_ACTIVE | 問わず | `ACT4_START` |
| INNER_ACTIVE | `forest_order` に 1 または 2 を含む | `ACT2_START` |
| INNER_ACTIVE | 上記以外 | `ACT2_START`（ACT3 は調整中のため代用） |

その他のボタン操作は現状ほぼコメントアウト済みです。

### 現状ほぼ未実装の mode

- `MODE2_POLE`
- `MODE7_SPEAR_ATTACK`


これらは関数自体は残っていますが、大半の操作がコメントアウトされています。

### `OperationMode / MODE9_AUTO_CHASSIS`

`MODE9_AUTO_CHASSIS` は、シャーシ自動シーケンスの開始要求を出すための操作ページです。実際の進行状態は `OperationMode` とは別に `auto_chassis_status_` で保持します。

### `chassis_act_status_ == NONE` かつ pending が無いときの操作

- `triangle`
  - `start_auto_chassis(ChassisAct::ACT0_START, {}, {})`
- `circle`
  - 青ゾーン用の開始姿勢を設定
  - `set_mecanum_yaw(0.0)`
  - `set_odometry(-5.5, 0.5, 0.0)`
  - `set_initialpose(-5.5, 0.5, 0.0)`
- `cross`
  - 内回り KFS 回収用の `RobotMove` を publish
  - `start_auto_chassis(ChassisAct::ACT2_START, forest_order, collect_kfs_type)`
  - 併せて内回り KFS 自動回収を開始
  - `forest_order` / `collect_kfs_type` は MODE3 → 次モード遷移時に確定した `kfs_auto_collect_plan_` を使います
- `square`
  - 外回り KFS 回収用の `RobotMove` を publish
  - `start_auto_chassis(ChassisAct::ACT4_START, forest_order, collect_kfs_type)`
  - 併せて外回り KFS 自動回収を開始
  - `forest_order` / `collect_kfs_type` は MODE3 → 次モード遷移時に確定した `kfs_auto_collect_plan_` を使います
- `down`
  - `start_auto_chassis(ChassisAct::ACT1_START, {}, {})`
- `right`
  - KFS 回収系の原点検出をまとめて実行

### 起動直後の開始要求

- `map -> base_link` TF がまだ無い間に `triangle` / `cross` / `square` / `down` を押した場合は、`RobotMove` を pending として保持します。
- TF が利用可能になった周期で、その pending 要求を 1 回だけ publish します。
- `MODE3_SPEAR` の pending 開始要求は `kfs_auto_collect_plan_.status` に応じて切り替わります。
  - `INNER_ACTIVE` のときは `ACT2_START`
  - `OUTER_ACTIVE` のときは `ACT4_START`
- `reset_robot()` 実行時は pending 要求を破棄します。
- `r2`
  - 実行中の `ACT*` を中断し、対応する `*_FINISH` を要求します。
  - pending の開始要求だけが残っている場合は、その pending を破棄します。

### 自動回収の挙動

- `kfs_auto_collect_status != NONE` の間は `map -> base_link` TF を見て、回収範囲の長方形に入ったかを判定します。
- `INNER_ACTIVE` では `inner_collect_kfs_center_pos.*` を、`OUTER_ACTIVE` では `outer_collect_kfs_center_pos.*` を使います。
- 判定対象の中心座標は `{zone}_inner_collect_kfs_center_pos.*` / `{zone}_outer_collect_kfs_center_pos.*` です（`zone` は `red` または `blue`）。
- 座標変換はコード側では行わず、YAML パラメータで各ゾーンの実座標を直接指定します。
- `collect_kfs_offset` を、使用する KFS 機構に応じて中心座標へ加えます。
- `kfs_yaw_delay_time` 秒だけ遅らせて、範囲外へ出た後の収納用 yaw 指令を送ります。
- `enable_auto_collect_kfs_actuator == false` のときは、範囲判定と LED 更新は続けますが、KFS の位置・yaw・ポンプ・バルブ指令は publish しません。
  - このパラメータは実行中の `ros2 param set` でも切り替えできます。
- 判定結果は LED `status` として反映します。
  - 複数の対象を見ている場合でも、どれか 1 つでも範囲内なら緑を優先
  - 範囲外なら赤
  - `base` 色は `OperationMode` に従います。
- `ACT1` / `ACT3` / `ACT5` は KFS 自動回収を起動せず、通常の軌道追従だけを行います。
- `ACT2` / `ACT4` が終了または中断されたときは、対応する KFS 自動回収も停止します。
- `ACT2` / `ACT3` / `ACT4` 実行中に 1 機でも KFS 回収ゾーン内にいる間は、`/chassis_tangent_pid_enable` に `false` を publish して `r1_chassis_control_node` の接線方向 PID を無効化します。これにより、ローカライズ誤差による追従暴走を防ぎつつ、法線方向・角度の PID は継続して機体の横ずれを抑制します。ゾーン外へ出ると `true` を publish して接線方向 PID を再有効化します。

### 制約

- `kfs_mechanism_type` の割り当ては実装上ほぼ青ゾーン前提です。
- 赤ゾーン側は TODO が残っており、未整備です。

## `reset_robot(bool is_start_zone)`

`reset_robot(is_start_zone)` では次を行います。

- 各 step カウンタを初期化
- `auto_chassis_status_` を `NONE` に戻し、pending のシャーシ自動要求を破棄
- `kfs_auto_collect_status` を `NONE` に戻し、KFS 自動回収の tracking 状態と収納用タイマを破棄
- メンバー変数 `zone_` を `blue` / `red` として検証します
- `zone_ == blue` なら開始姿勢を `(-5.5, 0.5, 0.0)` に設定します
- `zone_ == red` なら開始姿勢を `(5.5, 0.5, 0.0)` に設定します
- `/set_mecanum_yaw` と `/set_swerve_drive_yaw` に開始 yaw を送信します
- `/set_odometry` と `/initialpose` に開始姿勢を送信します
- 速度制御系とポンプ・バルブを停止
- 初期 state に戻します
- LED に 1 秒間の青点滅イベントを設定します
- この時点では `is_initialized_` を `true` に戻しません

`is_start_zone == false` の分岐は現状まだ TODO で、今は start zone と同じ開始姿勢を使います。

`/r1_machine_initialize` publish 自体は `reset_robot()` の外で行っており、`PS` ボタン押下時にセットで実行されます。
`PS` ボタン押下時には先に `is_initialized_ = false` に戻し、`activate_lidar_on_ps == true` なら LiDAR lifecycle node へ `activate` を要求してから `reset_robot(true)` を実行します。`reset_robot()` の中では自己位置や state は初期値へ戻しますが、`is_initialized_` は復帰させません。`/r1_machine_initialize_done` を受け取ったタイミングでだけ `is_initialized_ = true` へ復帰します。この完了通知では LED の再送キャッシュも無効化し、sabacan 初期化後に現在状態の LED 指令を再送できるようにしています。

LiDAR lifecycle の `activate` 要求は現在 state を確認せず、`urg_node2_1` / `urg_node2_2` の `/change_state` service へ送信します。すでに `active` の場合や `unconfigured` の場合、ROS 2 lifecycle の仕様上は要求が reject されることがあります。

## パラメータ

実際の bringup 設定は [`r1_machine_config.yaml`](../../r1_bringup/config/r1_machine_config.yaml) の `r1_main_node` セクションにあります。
bringup 起動時は [`r1_bringup.launch.py`](../../r1_bringup/launch/r1_bringup.launch.py) の `robot_control_mode` 引数を `r1_main_node` に渡します。

- `activate_lidar_on_ps`
  - `true` のとき、PS ボタン押下時に `lidar_lifecycle_node_names` の各 lifecycle node へ `activate` を要求します。
  - 既定値は `true` です。
- `lidar_lifecycle_node_names`
  - PS ボタン押下時の `activate` 要求対象です。
  - 既定値は `["urg_node2_1", "urg_node2_2"]` です。

### 基本

- `zone`
  - `blue` または `red`
- `cmd_vel_topic`
- `timer_rate`
- `ps4_connection_timeout`
- `enable_right_stick_pause`
  - `true` のとき、自動走行中（ACT2 / ACT3 / ACT4 / ACT5）に右スティック単押しで一時停止（ポーズ）できます。
  - 既定値は `false`。`false` のときは右スティック押し込みで従来通り自動走行を中断します。

### 足回り

- `chassis_max_velocity`
- `chassis_max_omega`

### KFS

- `use_kfs_mech_lock`
- `kfs_fx_normal_pos`
- `kfs_fx_storage_pos`
- `kfs_fx_start_pos`
- `kfs_fx_put_pos`
- `kfs_fx_expand_pos`
- `kfs_fx_low_mech_lock_pos`
- `kfs_fx_high_mech_lock_pos`
- `kfs_fz_normal_pos`
- `kfs_fz_low_pos`
- `kfs_fz_middle_pos`
- `kfs_fz_high_pos`
- `kfs_fz_put_pos`
- `kfs_fz_storage_pos`
- `kfs_fz_low_mech_lock_pos`
- `kfs_fz_high_mech_lock_pos`
- `kfs_fyaw_normal_angle`
- `kfs_fyaw_front_angle`
- `kfs_fyaw_side_angle`
- `kfs_fyaw_rear_angle`
- `kfs_fyaw_low_mech_lock_angle`
- `kfs_fyaw_high_mech_lock_angle`
- `kfs_rx_normal_pos`
- `kfs_rx_storage_pos`
- `kfs_rx_start_pos`
- `kfs_rx_put_pos`
- `kfs_rx_expand_pos`
- `kfs_rx_low_mech_lock_pos`
- `kfs_rx_high_mech_lock_pos`
- `kfs_rz_normal_pos`
- `kfs_rz_low_pos`
- `kfs_rz_middle_pos`
- `kfs_rz_high_pos`
- `kfs_rz_put_pos`
- `kfs_rz_storage_pos`
- `kfs_rz_low_mech_lock_pos`
- `kfs_rz_high_mech_lock_pos`
- `kfs_ryaw_normal_angle`
- `kfs_ryaw_front_angle`
- `kfs_ryaw_side_angle`
- `kfs_ryaw_rear_angle`
- `kfs_ryaw_low_mech_lock_angle`
- `kfs_ryaw_high_mech_lock_angle`

### R2 昇降

- `r2_lift_up_velocity`
- `r2_lift_down_velocity`

### /initialpose 再送

- `initialpose_retry1_delay_sec`
  - PS ボタン押下から最初の再送までの遅延 [s]
  - 既定値は `1.0`
- `initialpose_retry2_delay_sec`
  - PS ボタン押下から 2 回目の再送までの遅延 [s]
  - 既定値は `3.0`

### KFS 回収経路

- `collect_kfs_height`
- `collect_kfs_width`
- `collect_kfs_offset`
- `enable_auto_collect_kfs_actuator`
- `kfs_yaw_delay_time`
  - 範囲外へ出たあとに収納用 yaw を送るまでの遅延時間 [s]
  - 既定値は `1.0`
- `red_inner_collect_kfs_center_pos.<1..12>` / `blue_inner_collect_kfs_center_pos.<1..12>`
  - 内回り KFS 回収判定用長方形の中心座標 `[x, y, yaw]`。`zone` に応じたパラメータセットを使用します。
- `red_outer_collect_kfs_center_pos.<1..12>` / `blue_outer_collect_kfs_center_pos.<1..12>`
  - 外回り KFS 回収判定用長方形の中心座標 `[x, y, yaw]`。`zone` に応じたパラメータセットを使用します。

## 実装メモ

- `register_position_axis()` は `position_ref` publish、`detect_origin` publish、`mode_status` subscribe をまとめて登録します。
- `register_velocity_axis()` は `MotorRef` を `control_type = "VELOCITY"` で publish します。
- `publish_*` helper は publish と同時に内部の ref 値も更新します。
- `set_initialpose()` は引数の `delay_sec` 後に `/initialpose` を publish し、さらに `initialpose_retry1_delay_sec` 後・`initialpose_retry2_delay_sec` 後に計 2 回再送します。PS を連続で押した場合は既存のタイマーをすべてキャンセルしてから再スケジュールします。
- 初回 publish 後、`initialpose_tf_log_delay_sec` 秒待ってから `map->odom` と `map->base_link` の現在 TF を 1 回だけログします。既定値は `5.0` 秒です。
- `ps4_->is_connected() == false` の間は、`auto_chassis_status_` や KFS 自動回収状態に関係なく危険側のアクチュエータを停止します。

## Launch

- 通常の bringup では [`r1_bringup.launch.py`](../../r1_bringup/launch/r1_bringup.launch.py) から起動します。
- パラメータは [`r1_machine_config.yaml`](../../r1_bringup/config/r1_machine_config.yaml) から読み込みます。
- bringup では `cmd_vel_topic` を `/cmd_vel_target` に設定し、[`r1_chassis_velocity_control_node`](../../r1_control/docs/r1_chassis_velocity_control_node.md) を経由して最終的な `/cmd_vel` に変換されます。
- `robot_control_mode:=manual` なら `MainState=READY, OperationMode=MODE1_DETECT_ORIGIN, ChassisControlMode=MANUAL`、`robot_control_mode:=auto` なら `MainState=READY, OperationMode=MODE1_DETECT_ORIGIN, ChassisControlMode=AUTO` で起動します。

## 起動例

bringup から `MODE1_DETECT_ORIGIN` で起動:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch r1_bringup r1_bringup.launch.py robot_control_mode:=manual
```

bringup から足回り `AUTO` / `MODE1_DETECT_ORIGIN` で起動:

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
