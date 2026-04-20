# r1_aruco_serial_node

`r1_aruco_serial_node` は、`/aruco_marker_id` を受け取って小型ディスプレイ側へシリアル送信する ROS 2 ノードです。  
受信した `std_msgs/msg/Int32` の `data` を文字列化し、末尾に改行を付けて送ります。

PC 画面に PNG を表示する [r1_aruco_display_node](./r1_aruco_display_node.md) とは別に、外付け表示器へマーカ番号を渡したいときに使います。

## トピック

- Subscribe
  - `/aruco_marker_id` (`std_msgs/msg/Int32`)
    - 受信した `data` を表示対象の `marker_id` として使います。

## 主なパラメータ

| パラメータ名 | 型 | デフォルト値 | 説明 |
| --- | --- | --- | --- |
| `port` | string | なし | 接続先のシリアルデバイス名です。例: `/dev/ttyUSB0` |

## シリアル送信仕様

- 送信内容
  - `marker_id` を 10 進文字列化したものに改行 `\n` を付与して送信します。
  - 例: `12\n`
- ポート設定
  - ボーレート: `115200`
  - データ長: `8bit`
  - パリティ: `none`
  - ストップビット: `1`
- 受信処理
  - 100 ms 周期で受信をポーリングします。
  - 受信バッファに `\0` が含まれていた場合、その時点までの文字列をログへ出します。

## 動作概要

1. 起動時に `port` パラメータから接続先シリアルポートを開きます。
2. `/aruco_marker_id` を購読します。
3. `Int32.data` を受け取るたびに `"<marker_id>\n"` 形式の文字列を送信します。
4. 別タイマでシリアル受信をポーリングし、終端 `\0` を受けたときだけ受信内容をログに出します。

## ビルド

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/ros2_ws
colcon build --packages-select r1_ui
source install/setup.bash
```

## 起動例

```bash
source ~/ros2_ws/install/setup.bash
ros2 run r1_ui r1_aruco_serial_node --ros-args -p port:=/dev/ttyUSB0
```

## 送信確認例

一度だけ送る:

```bash
ros2 topic pub --once /aruco_marker_id std_msgs/msg/Int32 "{data: 12}"
```

1 Hz で送り続ける:

```bash
ros2 topic pub /aruco_marker_id std_msgs/msg/Int32 "{data: 3}" -r 1
```

## 注意点

- `port` パラメータ未指定のまま起動すると、正しいデバイスを開けず初期化に失敗する可能性があります。
- 送信フォーマットは単純な ASCII 文字列です。受信側ファームが別形式を要求する場合は、このノード側を合わせて変更してください。
- 受信ログは `\0` 終端前提です。受信側が null 終端を返さない場合、現在の実装ではログ出力されません。
- 受信バッファ長は 256 byte 固定です。長い応答を扱う場合はオーバーフロー対策を追加したほうが安全です。
