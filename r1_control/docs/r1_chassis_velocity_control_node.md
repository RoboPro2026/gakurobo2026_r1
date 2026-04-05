# r1_chassis_velocity_control_node

`r1_chassis_velocity_control_node` は、`input_cmd_vel_topic` と実測オドメトリ `odometry` を用いて、`output_cmd_vel_topic` に補正後の速度指令を生成する ROS 2 ノードです。実装は PID ですが、`kd_* = 0.0` にして PI 制御として使う前提です。

bringup では、`r1_main_node` と `r1_chassis_control_node` が出す `/cmd_vel_target` をこのノードが受け、補正後の `/cmd_vel` を `r1_swerve_drive_node` や `r1_dummy_odometry_node` に渡します。

## トピック

- Subscribe
  - `input_cmd_vel_topic` (`geometry_msgs/msg/Twist`): 補正前の速度指令
  - `odometry` (`nav_msgs/msg/Odometry`): 実測車体速度を含むオドメトリ
  - `/chassis_velocity_control_initialize` (`std_msgs/msg/Empty`): PID 計算用の内部状態をリセットする初期化信号
- Publish
  - `output_cmd_vel_topic` (`geometry_msgs/msg/Twist`): 補正後の速度指令

## 主なパラメータ

| パラメータ名 | 型 | デフォルト値 | 説明 |
| --- | --- | --- | --- |
| `enable_velocity_pid` | bool | `false` | PID 補正を有効にするか。`false` のときは `input_cmd_vel` をそのまま通します。 |
| `control_rate` | double | `100.0` | 制御周期 [Hz]。 |
| `input_cmd_vel_topic` | string | `"/cmd_vel_target"` | 補正前の速度指令を subscribe する topic 名。 |
| `output_cmd_vel_topic` | string | `"/cmd_vel"` | 補正後の速度指令を publish する topic 名。 |
| `kp_vx`, `ki_vx`, `kd_vx` | double | `0.0` | `linear.x` 方向の PID ゲイン。 |
| `kp_vy`, `ki_vy`, `kd_vy` | double | `0.0` | `linear.y` 方向の PID ゲイン。 |
| `kp_omega`, `ki_omega`, `kd_omega` | double | `0.0` | `angular.z` 方向の PID ゲイン。 |
| `integral_limit_vx`, `integral_limit_vy`, `integral_limit_omega` | double | `0.5` | 各軸の積分状態の飽和上限です。 |
| `output_limit_vx`, `output_limit_vy`, `output_limit_omega` | double | `2.5` | 補正後に publish する速度指令の上限です。 |

## 制御の流れ

1. `input_cmd_vel_topic` から受けた速度指令を保持する。
2. `odometry.twist.twist` を実測速度として保持する。
3. `enable_velocity_pid = false` のときは `input_cmd_vel_topic` から受けた値をそのまま `output_cmd_vel_topic` に流す。
4. `enable_velocity_pid = true` のときは、各軸で `error = reference - measured` を計算し、`reference + PID(error)` を出力する。
5. `kd_* = 0.0` にして使えば PI 制御になる。
6. `input_cmd_vel_topic` と `odometry` は最後に受けた値を保持して使う。
7. `odometry` をまだ受けていない間は補正を止め、入力指令をそのまま通す。
8. `/chassis_velocity_control_initialize` を受けたときは、積分項・前回誤差・制御周期計算用時刻をリセットする。

## bringup での接続

- `r1_main_node`: `cmd_vel_topic = "/cmd_vel_target"`
- `r1_chassis_control_node`: `cmd_vel_topic = "/cmd_vel_target"`
- `r1_chassis_velocity_control_node`: `input_cmd_vel_topic = "/cmd_vel_target"`, `output_cmd_vel_topic = "/cmd_vel"`
- `r1_machine_manage_node`: `/chassis_velocity_control_initialize` を publish
- `r1_swerve_drive_node`: `/cmd_vel` を subscribe

## 起動例

単体起動:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run r1_control r1_chassis_velocity_control_node --ros-args \
  -p enable_velocity_pid:=true \
  -p kp_vx:=0.5 -p ki_vx:=0.1 \
  -p kp_vy:=0.5 -p ki_vy:=0.1 \
  -p kp_omega:=0.3 -p ki_omega:=0.05
```
