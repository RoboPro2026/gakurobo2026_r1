# remote_debug_client.py

[`remote_debug_node.py`](remote_debug_node.md) を操作するクライアントスクリプトです。rosbridge WebSocket サーバーに接続してトピックを操作します。

`session` / `start` / `monitor` コマンドは常駐し、`/r1_rosout_bridge` を受信して画面に出力します。`Ctrl+C` で停止します。

## 依存

```bash
pip install roslibpy
```

## 接続先

`--host` で rosbridge サーバーのホストを指定します。省略時は `localhost`（デフォルトポート: 9090）。

miniPC の rosbridge に接続する場合は miniPC の IP アドレスを指定してください。

```bash
ros2 run r1_bringup remote_debug_client.py monitor --host 192.168.50.12
```

## コマンド

### `monitor`

rosout 有効化 → 常駐（録画なし）

録画は不要でロボットのログだけ確認したい場合に使います。`Ctrl+C` で rosout 無効化して終了します。

```bash
ros2 run r1_bringup remote_debug_client.py monitor
ros2 run r1_bringup remote_debug_client.py monitor --host 192.168.50.12
```

### `session`

rosout 有効化 → 待機 → 録画開始 → 常駐

デバッグセッション開始時の標準手順です。`Ctrl+C` で録画停止・rosout 無効化して終了します。

```bash
ros2 run r1_bringup remote_debug_client.py session
ros2 run r1_bringup remote_debug_client.py session --host 192.168.50.12 --bag 2026-05-17_match1
ros2 run r1_bringup remote_debug_client.py session --host 192.168.50.12 --bag 2026-05-17_match1 --delay 2.0
```

| オプション | 既定値 | 説明 |
|---|---|---|
| `--bag` | （自動生成） | バッグ名 |
| `--delay` | `1.0` | rosout 有効化から録画開始までの待機時間 [s] |

### `start`

録画開始（rosout は有効化しない）→ 常駐

rosout を垂れ流したくない場合に使います。`Ctrl+C` で録画停止して終了します。

```bash
ros2 run r1_bringup remote_debug_client.py start
ros2 run r1_bringup remote_debug_client.py start --host 192.168.50.12 --bag 2026-05-17_match1
```

| オプション | 既定値 | 説明 |
|---|---|---|
| `--bag` | （自動生成） | バッグ名 |

### `stop`

録画を停止して終了します（one-shot）。

```bash
ros2 run r1_bringup remote_debug_client.py stop
ros2 run r1_bringup remote_debug_client.py stop --host 192.168.50.12
```

### `rosout-on` / `rosout-off`

rosout 転送を有効化・無効化して終了します（one-shot）。

```bash
ros2 run r1_bringup remote_debug_client.py rosout-on --host 192.168.50.12
ros2 run r1_bringup remote_debug_client.py rosout-off --host 192.168.50.12
```

## 共通オプション

| オプション | 既定値 | 説明 |
|---|---|---|
| `--host` | `localhost` | rosbridge WebSocket ホスト |
| `--port` | `9090` | rosbridge WebSocket ポート |

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

WARN は黄色、ERROR / FATAL は赤色で表示されます。

## rosbridge のポート

rosbridge はデフォルトポート `9090` で起動します。`r1_bringup.launch.py` を `use_phone:=true` で起動している場合、rosbridge はロボット側ですでに動いています。複数クライアント（ThinkPad・スマホなど）から同時接続可能です。
