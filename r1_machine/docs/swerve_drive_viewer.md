# swerve_drive_viewer

`swerve_drive_viewer.py` は、`/swerve_drive_ref`（各輪の速度ベクトル）をXY平面上に可視化し、右ペインのGUIから `/cmd_vel` と `/bno086/imu/data_raw` を送信できるデバッグ用ビューアです。

- スクリプト: `src/gakurobo2026_r1/r1_machine/example/swerve_drive_viewer.py`
- 可視化:
  - 各輪の `omega, theta` を矢印で表示（`wheel_radius` で線速度へ換算）
  - ロボット外形（矩形）とホイール位置をYawで回転表示（`rotate_robot`）
  - `cmd_vel` の並進（x,y）を1本の矢印、回転（omega）を円弧矢印で表示
- GUI送信:
  - `/cmd_vel`（`vx, vy, omega`）
  - `/bno086/imu/data_raw`（Yawのみ。roll/pitch=0、角速度/加速度=0）

## トピック

- **Subscribe**
  - `/swerve_drive_ref` (`r1_msgs/msg/SwerveDrive`)
  - `/cmd_vel` (`geometry_msgs/msg/Twist`)
  - `/bno086/imu/data_raw` (`sensor_msgs/msg/Imu`)（ロボット回転表示用。`imu_sub_topic`）
- **Publish（GUI）**
  - `/cmd_vel` (`geometry_msgs/msg/Twist`)（`cmd_vel_pub_topic`）
  - `/bno086/imu/data_raw` (`sensor_msgs/msg/Imu`)（`imu_pub_topic`）

## 主なパラメータ

| パラメータ名 | 型 | デフォルト値 | 説明 |
| --- | --- | --- | --- |
| `topic` | string | `/swerve_drive_ref` | 可視化する `SwerveDrive` の購読先。 |
| `cmd_vel_topic` | string | `/cmd_vel` | 受信表示に使う `cmd_vel` の購読先。 |
| `cmd_vel_pub_topic` | string | `/cmd_vel` | GUIから `cmd_vel` をPublishする先。 |
| `imu_sub_topic` | string | `/bno086/imu/data_raw` | ロボット回転表示に使うIMU購読先。 |
| `imu_pub_topic` | string | `/bno086/imu/data_raw` | GUIからIMUをPublishする先。 |
| `rotate_robot` | bool | `true` | IMU（優先）または `cmd_vel` 積分Yawでロボット外形/ホイール位置を回転表示。 |
| `wheel_radius` | double | `0.1` | `swerve_v_is_angular=true` のとき、`/swerve_drive_ref` の `omega*` [rad/s] を `v_linear = wheel_radius * omega*` [m/s] に変換して表示します。 |
| `swerve_v_is_angular` | bool | `true` | `/swerve_drive_ref` の `omega*` を角速度 [rad/s] として扱い、`wheel_radius` で線速度 [m/s] に変換してプロットします（旧データ互換で `false` にすると入力を線速度扱い）。 |
| `robot_length` | double | `0.5` | 可視化用ロボット長さ [m]。 |
| `robot_width` | double | `0.5` | 可視化用ロボット幅 [m]。 |
| `vector_scale` | double | `0.3` | 各輪ベクトル表示のスケール。 |
| `cmd_vel_vector_scale` | double | `0.3` | `cmd_vel` 並進矢印のスケール。 |
| `omega_arc_radius_scale` | double | `0.45` | `omega` 円弧矢印の半径（ロボット寸法に対する比）。 |
| `rate_hz` | double | `30.0` | 描画更新周期 [Hz]。 |
| `enable_cmd_vel_publish_ui` | bool | `true` | `cmd_vel` 送信UIを表示/有効化。 |
| `cmd_vel_publish_rate_hz` | double | `20.0` | `cmd_vel` Auto pub の送信周期 [Hz]。 |
| `cmd_vel_limit_vx` | double | `2.0` | `vx` 入力のクランプ上限（±）。 |
| `cmd_vel_limit_vy` | double | `2.0` | `vy` 入力のクランプ上限（±）。 |
| `cmd_vel_limit_omega` | double | `6.28` | `omega` 入力のクランプ上限（±）。 |
| `enable_imu_publish_ui` | bool | `true` | IMU(Yaw) 送信UIを表示/有効化。 |
| `imu_publish_rate_hz` | double | `50.0` | IMU Auto pub の送信周期 [Hz]。 |
| `imu_yaw_limit` | double | `pi` | Yaw 入力のクランプ上限（±）。 |
| `imu_frame_id` | string | `""` | PublishするIMUメッセージの `header.frame_id`。 |
| `always_on_top` | bool | `false` | 可能なバックエンドで最前面表示を設定。 |
| `raise_window` | bool | `false` | `matplotlib` の `figure.raise_window` を制御。 |

## 起動例

ターミナル1
```bash
source ~/ros2_ws/install/setup.bash
ros2 run r1_machine r1_swerve_drive_node
```

ターミナル2
```bash
source ~/ros2_ws/install/setup.bash
python3 src/gakurobo2026_r1/r1_machine/example/swerve_drive_viewer.py
```

## 凡例（グラフ）

- **黒い矩形**: ロボット外形（`robot_length` × `robot_width`）。`rotate_robot=true` のときYawで回転します。
- **オレンジの点 + 数字(0〜3)**: ホイール位置とホイール番号。
- **青い矢印（4本）**: `/swerve_drive_ref` の各輪ベクトル（長さ= `omega*` を `wheel_radius` で線速度へ変換した値、向き= `theta*` をXYに投影して表示。表示倍率は `vector_scale`）。
- **赤い矢印（1本）**: `/cmd_vel` の並進（`linear.x`, `linear.y`）を原点から1本のベクトルで表示（表示倍率は `cmd_vel_vector_scale`）。
- **赤い円弧 + 矢印**: `/cmd_vel` の回転（`angular.z`）。符号で回転方向、|omega|で弧の長さ/濃さが変わります（`omega_arc_radius_scale`）。

パラメータを指定する例:

```bash
python3 src/gakurobo2026_r1/r1_machine/example/swerve_drive_viewer.py --ros-args \
  -p robot_length:=0.6 -p robot_width:=0.4 \
  -p topic:=/swerve_drive_ref \
  -p cmd_vel_pub_topic:=/cmd_vel \
  -p imu_pub_topic:=/bno086/imu/data_raw
```
