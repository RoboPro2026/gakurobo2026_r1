# r1_sabacan_msgs_converter_node

`r1_sabacan_msgs_converter_node` は、`sabacan_msgs` / `sabacan_single_control_msgs` と `r1_msgs` の橋渡しを行う ROS 2 ノードです。Sabacan 側から受けたモータ・GPIO 状態を `r1_msgs` の各トピックへ再配信し、逆に `r1_msgs` の指令を Sabacan 用のトピックへ変換して送ります。状態 publish は系統ごとに個別の `timer_rate` で設定できます。

## 主な役割

- **Sabacan → R1**
  - `/sabacan_robomas_status*` を受け取り、メカナム・オドメトリエンコーダ・各機構の `r1_msgs` ステータスへ反映
  - `/sabacan_gpio_status*` を受け取り、各種リミットスイッチの `r1_msgs/msg/GpioInput` へ反映
- **R1 → Sabacan**
  - `r1_msgs/msg/MotorRef` を `sabacan_single_control_msgs/msg/SabacanRobomasSingleRef` へ変換
  - `r1_msgs/msg/GpioPwmRef` / `r1_msgs/msg/GpioServoRef` を `sabacan_msgs/msg/SabacanGPIORefFloat` / `SabacanGPIORefInt` へ変換
- **Debug**
  - 一部のモータ・GPIO 入力は `/debug_*` トピックにもそのまま publish

## 主な入出力トピック

トピック数が多いため、代表的なものだけ列挙します。

- **Subscribe**
  - `/sabacan_robomas_status0`〜`/sabacan_robomas_status9` (`sabacan_msgs/msg/SabacanRobomasStatus`)
  - `/sabacan_gpio_status0`〜`/sabacan_gpio_status9` (`sabacan_msgs/msg/SabacanGPIOStatus`)
  - `/mecanum_wheel_speeds_ref` (`r1_msgs/msg/Mecanum`)
  - `/kfs_*_motor_ref`, `/front_expand_motor_ref`, `/rear_expand_motor_ref`, `/r2_lift_motor_ref`, `/pole_*_motor_ref`, `/spear_*_motor_ref` (`r1_msgs/msg/MotorRef`)
  - `/kfs_*_gpio_pwm_ref`, `/pole_valve*_gpio_pwm_ref`, `/spear_hand_valve*_gpio_pwm_ref`, `/brake_valve_gpio_pwm_ref` (`r1_msgs/msg/GpioPwmRef`)
  - `/pole_servo*_gpio_servo_ref` (`r1_msgs/msg/GpioServoRef`)
- **Publish**
  - `/mecanum_wheel_speeds_feedback` (`r1_msgs/msg/Mecanum`)
  - `/odometry_encoder` (`r1_msgs/msg/OdometryEncoder`)
  - `/kfs_*_linear_motion_status`, `/kfs_*_angle_motion_status`
  - `/front_expand_linear_motion_status`, `/rear_expand_linear_motion_status`
  - `/r2_lift_motor_status`
  - `/pole_*_linear_motion_status`
  - `/spear_*_linear_motion_status`, `/spear_rotate_angle_motion_status`
  - `/kfs_*_switch_status`, `/spear_*_switch_status`
  - `/sabacan_robomas_ref<board_id>/motor<motor_number>`
  - `/sabacan_gpio_ref_float<board_id>`, `/sabacan_gpio_ref_int<board_id>`

## timer_rate パラメータ

各系統の周期 publish は独立したタイマーで動作します。`0.0` 以下を指定すると、その系統の周期 publish を停止します。

| パラメータ名 | 型 | デフォルト値 | 対象 |
| --- | --- | --- | --- |
| `mecanum_timer_rate` | double | `100.0` | `/mecanum_wheel_speeds_feedback` |
| `odometry_encoder_timer_rate` | double | `100.0` | `/odometry_encoder` |
| `kfs_timer_rate` | double | `100.0` | KFS 回収機構の各 `*_status` |
| `expand_timer_rate` | double | `100.0` | 展開機構の `*_linear_motion_status` |
| `r2_lift_timer_rate` | double | `100.0` | `/r2_lift_motor_status` |
| `pole_timer_rate` | double | `100.0` | ポール回収機構の各 `*_status` |
| `spear_timer_rate` | double | `100.0` | やり機構の各 `*_status` |
| `gpio_timer_rate` | double | `100.0` | 各種スイッチ入力の `*_switch_status` |

## 動作概要

1. Sabacan の状態トピックを受信すると、対応する内部状態を更新します。
2. `r1_msgs` 側から来た指令は、その場で対応する Sabacan トピックへ変換して publish します。
3. 状態系は系統ごとのタイマーで内部状態を再送信します。
4. デバッグ用トピックは Sabacan 受信コールバック内で即時 publish されます。

## 起動例

```bash
source ~/ros2_ws/install/setup.bash
ros2 run r1_machine r1_sabacan_msgs_converter_node --ros-args \
  -p mecanum_timer_rate:=100.0 \
  -p odometry_encoder_timer_rate:=50.0 \
  -p gpio_timer_rate:=20.0
```

## 補足

- 配線や board / motor / pin の対応はソースコード内の `BoardInfo` 定義で固定されています。
- 新しい機構を追加するときは、対応する `BoardInfo`、publisher/subscriber、状態反映処理、周期 publish 関数のいずれかを更新してください。
