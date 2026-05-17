# remote_debug_client.py

[`remote_debug_node.py`](remote_debug_node.md) を操作するクライアントスクリプトです。`ros2 topic pub` では DDS の遅延によりトピックが届かない場合があるため、rclpy でサブスクライバーの存在を確認してから publish します。

`session` / `start` コマンドは rosbridge を起動して常駐し、`/r1_rosout_bridge` を受信して画面に出力します。`Ctrl+C` で録画・rosbridge を停止して終了します。

## コマンド

### `session`

rosbridge 起動 → rosout 有効化 → 待機 → 録画開始 → 常駐

デバッグセッション開始時の標準手順です。`Ctrl+C` で録画停止・rosout 無効化・rosbridge 停止を行って終了します。

```bash
ros2 run r1_bringup remote_debug_client.py session
ros2 run r1_bringup remote_debug_client.py session --bag 2026-05-17_match1
ros2 run r1_bringup remote_debug_client.py session --bag 2026-05-17_match1 --delay 2.0
```

| オプション | 既定値 | 説明 |
|---|---|---|
| `--bag` | （自動生成） | バッグ名 |
| `--delay` | `1.0` | rosout 有効化から録画開始までの待機時間 [s] |
| `--rosbridge-delay` | `1.0` | rosbridge 起動後の待機時間 [s] |

### `start`

rosbridge 起動 → 録画開始（rosout は有効化しない）→ 常駐

rosout を垂れ流したくない場合に使います。`Ctrl+C` で録画停止・rosbridge 停止を行って終了します。

```bash
ros2 run r1_bringup remote_debug_client.py start
ros2 run r1_bringup remote_debug_client.py start --bag 2026-05-17_match1
```

| オプション | 既定値 | 説明 |
|---|---|---|
| `--bag` | （自動生成） | バッグ名 |
| `--rosbridge-delay` | `1.0` | rosbridge 起動後の待機時間 [s] |

### `stop`

録画を停止して終了します（one-shot、rosbridge は起動しません）。

```bash
ros2 run r1_bringup remote_debug_client.py stop
```

### `rosout-on` / `rosout-off`

rosout 転送を有効化・無効化して終了します（one-shot、rosbridge は起動しません）。

```bash
ros2 run r1_bringup remote_debug_client.py rosout-on
ros2 run r1_bringup remote_debug_client.py rosout-off
```

## rosout の出力形式

`/r1_rosout_bridge` を受信すると標準出力に以下の形式で出力します。

```
[LEVEL] [node_name]: message
```

例：

```
[INFO] [r1_main_node]: kfs_fx pos 0.100000
[WARN] [r1_main_node]: Already recording, ignoring start request
[ERROR] [r1_main_node]: kfs_fx position axis is not initialized
```

## rosbridge のポート

rosbridge はデフォルトポート `9090` で起動します。MSI Claw からは `ws://<robot_ip>:9090` で接続します。`r1_bringup.launch.py` を `use_phone:=false`（既定値）で起動している場合は競合しません。
