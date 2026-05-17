# remote_debug_node.py

MSI Claw から rosbridge 経由でロボットの状態を遠隔デバッグするためのノードです。`r1_bringup.launch.py` によって常時起動されます。操作は [`remote_debug_client.py`](remote_debug_client.md) または rosbridge 経由のトピックで行います。

## 主な役割

- `/record_start` を受信して `ros2 bag record` を起動する
- `/record_stop` を受信して録画を停止する
- `/enable_publish_rosout` で制御される `/rosout` → `/r1_rosout_bridge` の転送を行う
  - 転送は 200ms ごとにバッファをまとめて送信（rosbridge への負荷軽減のため）

## トピック

### Subscribe

| トピック名 | 型 | 説明 |
|---|---|---|
| `/record_start` | `std_msgs/String` | `data` にバッグ名を入れて録画開始。空文字なら自動名 |
| `/record_stop` | `std_msgs/Empty` | 録画停止 |
| `/enable_publish_rosout` | `std_msgs/Bool` | `true` で rosout 転送開始、`false` で停止 |
| `/rosout` | `rcl_interfaces/Log` | ROS ログ集約トピック（全ノード共通） |

### Publish

| トピック名 | 型 | 説明 |
|---|---|---|
| `/r1_rosout_bridge` | `rcl_interfaces/Log` | rosbridge 経由で MSI Claw から購読する転送先 |

## 録画の仕様

- `ros2 bag record -a --exclude <CAN_TOPIC_REGEX>` を実行する
- CAN 系トピック（`sabacan_*`、`from_can_bus*`、`to_can_bus*`）は除外される
- 作業ディレクトリは `~/ros2_ws`
- バッグ名を省略した場合は `ros2 bag` が自動生成する名前を使用する

## rosout 転送の仕様

- `/enable_publish_rosout` が `false`（既定値）の間は `/rosout` を購読しても転送しない
- 有効化されると `/rosout` に届いたメッセージを 200ms ごとにまとめて `/r1_rosout_bridge` に転送する
- 試合中など不要なときは `false` のままにしておくことで rosbridge への負荷をゼロにできる
