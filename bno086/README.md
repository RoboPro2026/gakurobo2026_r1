# BNO086 ROS 2 ノード (`bno086`)

このパッケージは、BNO086 IMUセンサーからシリアル通信（UART-RVCモード）でデータを読み取り、ROS 2の標準的な `sensor_msgs/msg/Imu` メッセージとしてパブリッシュするノードを提供します。

センサーのYaw（ヨー）、Pitch（ピッチ）、Roll（ロール）、および線形加速度は、ROS 2の標準座標系（REP 103: X-Forward, Y-Left, Z-Up）に準拠するように変換されてからパブリッシュされます。

## ⚠️ 1. 必須のセットアップ（パーミッション設定）

`bno086_node` はシリアルポート（例: `/dev/ttyACM0`、`/dev/ttyUSB0`）にアクセスする必要があります。デフォルトでは、Linuxは一般ユーザーにこのデバイスへの書き込み権限を与えていません。

ノードを実行する前に、**必ず**以下のコマンドを実行して、シリアルポートにアクセスできるようにしてください。

**方法A: ユーザーを `dialout` グループに追加する（推奨）**

`dialout` グループ（シリアルポートにアクセスできるグループ）にカレントユーザーを追加します。

```bash
sudo usermod -a -G dialout $USER
```

**注意:** このコマンドの実行後、設定を反映させるために**一度PCを再起動**するか、**ログアウトして再度ログイン**する必要があります。

**方法B: デバイスの権限を一時的に変更する**

これはPCを再起動するか、デバイスを抜き差しするたびに元に戻ります。

```bash
sudo chmod 666 /dev/ttyACM0
```

(ポートが異なる場合は `/dev/ttyACM0` の部分を適宜変更してください)

## 2\. FTDIのドライバの場合
FTDIのドライバの場合、データがデフォルトだと16ms周期でしか送信されません。  
なので、レイテンシを1msに書き換えます。  
レイテンシの確認方法
```bash
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

レイテンシの書き換え

```bash
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

## 3\. ノードの詳細

### パブリッシュするトピック

  * **`/bno086/imu/data_raw`** (`sensor_msgs/msg/Imu`)
      * センサーから取得したIMUデータ（向き、角速度、線形加速度）をパブリッシュします。
      * `header.frame_id` は `bno086_link` に設定されます。

### 必要なパラメータ

  * **`port`** (`std::string`)
      * BNO086が接続されているシリアルポートのパス。
      * **このパラメータは起動時に必ず指定する必要があります**。
      * 例: `"/dev/ttyACM0"`

### 動的に変更可能なパラメータ（オフセット）

ノードの実行中に `ros2 param set` コマンドで変更可能なオフセット値です。ロボットの初期位置の設定などに使うことを想定しています。

  * `offset_roll_angle` (double、初期値は0.0)
  * `offset_pitch_angle` (double、初期値は0.0)
  * `offset_yaw_angle` (double、初期値は0.0)
  * `offset_roll_angular_velocity` (double、初期値は0.0)
  * `offset_pitch_angular_velocity` (double、初期値は0.0)
  * `offset_yaw_angular_velocity` (double、初期値は0.0)
  * `offset_x_axis_accel` (double、初期値は0.0)
  * `offset_y_axis_accel` (double、初期値は0.0)
  * `offset_z_axis_accel` (double、初期値は0.0)

## 4\. 使い方と具体例

### 例1: ノードの基本実行

`port` パラメータを指定してノードを実行します。

```bash
ros2 run bno086 bno086_node --ros-args -p port:="/dev/ttyACM0"
```

### 例2: 起動時にオフセットを指定して実行

起動時から `yaw` の角度に `0.1` ラジアンのオフセットを加える場合：

```bash
ros2 run bno086 bno086_node --ros-args -p port:="/dev/ttyACM0" -p offset_yaw_angle:=0.1
```

### 例3: パブリッシュされたIMUデータをターミナルで確認

```bash
ros2 topic echo /bno086/imu/data_raw
```

### 例4: 実行中にオフセットを動的に変更

ノードが実行されている別ターミナルで以下のコマンドを実行すると、`bno086_node` の `offset_yaw_angle` が `0.05` ラジアンに設定されます。

```bash
ros2 param set /bno086_node offset_yaw_angle 0.05
```

### 例5: C++でIMUデータを購読する（サンプルコード）

`bno086/example/bno086_subscription_node.cpp` は、`/bno086/imu/data_raw` トピックを購読し、受け取ったクォータニオンをRPY（オイラー角）に変換してログに出力するサンプルノードです。

このサンプルノードは `bno086_node` とは別に実行できます。

```bash
ros2 run bno086 bno086_subscription_node
```

**`bno086_subscription_node.cpp` の主要部分:**

```cpp
// (ヘッダーファイル省略)

class MyNode : public rclcpp::Node
{
public:
  MyNode() : Node("bno086_subscription_node")
  {
    // "bno086/imu/data_raw" トピックを購読
    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "bno086/imu/data_raw", 10, std::bind(&MyNode::topic_callback, this, std::placeholders::_1));
  }

private:
  void topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // 受け取ったクォータニオン(msg->orientation)をtf2::Quaternionに変換
    tf2::Quaternion q(
      msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    
    // tf2::Matrix3x3 を使って RPY (Roll, Pitch, Yaw) に変換
    double yaw, pitch, roll;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    
    RCLCPP_INFO(this->get_logger(), "Yaw: %.3f, Pitch: %.23, Roll: %.3f", yaw, pitch, roll);
    // (角速度と加速度のログ出力は省略)
  }
  // (以下省略)
};
```