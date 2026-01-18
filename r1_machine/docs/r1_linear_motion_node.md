# r1_linear_motion_node

`r1_linear_motion_node` はリニア機構用の Robomas モータを制御する ROS 2 ノードです。`/linear_motion_status` で得たトルク・速度・位置とリミットスイッチ入力を監視しながら、通常は位置指令を `/linear_motion_motor_ref` に出力します。原点検出要求が入った場合は一定速度で巻き取り/押し出しを行い、リミットスイッチまたはトルク上昇で停止して位置オフセットを更新します。目標位置はメートル単位で受け取り、ドラム半径 `radius` を用いてモータ角度へ換算します。

## トピック

- **Subscribe**
  - `/linear_motion_status` (`r1_msgs/msg/LinearMotion`): トルク [Nm], 速度 [rad/s], 位置 [rad] を取得。
  - `/low_switch_status`(`std_msgs/msg/Bool`): スイッチの値。 `inverse_*data` で XOR 反転されます。
  - `/high_switch_status`(`std_msgs/msg/Bool`): スイッチの値。 `inverse_*data` で XOR 反転されます。
  - `/linear_motion_positon_ref` (`std_msgs/msg/Float64`): 目標位置 [m]。原点検出中（速度モード）は無視されます。
  - `/linear_motion_detect_origin` (`std_msgs/msg/Bool`): `true` で原点検出モードに移行し、`false` で通常の位置モードに戻ります。
- **Publish**
  - `/linear_motion_motor_ref` (`r1_msgs/msg/MotorRef`): Robomas へ渡す制御指令。`control_type` は `"POSITION"` または `"VELOCITY"`、`ref` は角度 [rad] もしくは角速度 [rad/s]。

## 主なパラメータ

`declare_parameter` で定義されたデフォルト値と役割は次の通りです。`add_on_set_parameters_callback` を使っているため、`ros2 param set` で実行中に更新できます。

| パラメータ名 | 型 | デフォルト値 | 説明 |
| --- | --- | --- | --- |
| `use_low_switch` | bool | `true` | 低側リミットスイッチを原点検出判定に使うか。 |
| `use_high_switch` | bool | `true` | 高側リミットスイッチを原点検出判定に使うか。 |
| `torque_threshold` | double | `1.0` | 許容トルク [Nm]。この絶対値を超え続けたら原点とみなします。 |
| `origin_detect_threshold_time` | double | `0.1` | トルクしきい値超過を原点とみなすまでの継続時間 [s]。 |
| `origin_detect_speed` | double | `-3.14` | 原点検出中に流す一定角速度 [rad/s]。負の符号でも可。  符号は `inverse_motor` に応じて反転します。 |
| `pos_min` | double | `0.0` | 指令として受け付ける位置の下限 [m]。外れた場合はクランプされます。 |
| `pos_max` | double | `1.0` | 指令として受け付ける位置の上限 [m]。 |
| `radius` | double | `0.05` | 巻取りドラムの半径 [m]。位置[m] をモータ角度[rad]に換算するときに使用。 |
| `inverse_motor` | bool | `false` | モータ正転方向を反転するフラグ。`true` で `ref` に -1 を掛けます。 |
| `inverse_low_switch_logic` | bool | `false` | 低側リミットスイッチの論理を反転します。 |
| `inverse_high_switch_logic` | bool | `false` | 高側リミットスイッチの論理を反転します。 |

## 動作の流れ

1. `/linear_motion_status` を受信してトルク・速度・位置・リミットスイッチ状態を内部に保持します。スイッチ値は `inverse_*_logic` 設定で XOR 反転されます。
2. 通常は位置モード（`MODE_POSITION`）。`/linear_motion_positon_ref` で受けた目標位置 [m] を `pos_min`〜`pos_max` にクランプし、原点検出で決まる `pos_offset` を加算した後、`target_angle = (target_pos) / radius` として `"POSITION"` 指令を配信します (`inverse_motor` で符号反転)。速度モード中はこのトピックを無視します。
3. `/linear_motion_detect_origin` に `true` を送ると速度モード（原点検出）へ移行し、10 ms 周期のタイマで `"VELOCITY"` 指令 `-origin_detect_speed` を流し続けます。検出条件は以下の OR です。  
   - `use_low_switch`/`use_high_switch` が有効で、対応するスイッチがオン。  
   - `|torque| > torque_threshold` の状態が `origin_detect_threshold_time` 秒以上続く。
4. 検出条件を満たすと、現在の `pos` から `pos_offset = radius * pos` を設定し、`"POSITION"` 指令でその場に停止したうえで位置モードへ復帰します。`/linear_motion_detect_origin` に `false` を送れば手動でも位置モードへ戻せます（オフセット更新は行いません）。

## 起動と利用例

- 単体起動例:

  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 run r1_machine r1_linear_motion_node
  ```

- 原点検出を開始する:

  ```bash
  ros2 topic pub /linear_motion_detect_origin std_msgs/Bool '{data: true}'
  ```

- 原点決め後に 0.2 m へ移動させる指令:

  ```bash
  ros2 topic pub /linear_motion_positon_ref std_msgs/Float64 '{data: 0.2}'
  ```

- 状態確認: `ros2 topic echo /linear_motion_status`。トルクがしきい値を超え続けていないか、リミットスイッチが正しく反応しているかをここで確認できます。
