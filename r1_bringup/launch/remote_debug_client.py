#!/usr/bin/env python3
"""
remote_debug_node を制御するクライアント。
rosbridge WebSocket サーバーに接続してトピックを操作する。

Usage:
  ros2 run r1_bringup remote_debug_client.py monitor [--host HOST] [--port PORT]
      rosout 有効化 → 常駐（録画なし）→ Ctrl+C で全停止。

  ros2 run r1_bringup remote_debug_client.py session [--bag BAG_NAME] [--delay SEC] [--host HOST] [--port PORT]
      rosout 有効化 → --delay 秒待機 → 録画開始 → Ctrl+C で全停止。

  ros2 run r1_bringup remote_debug_client.py start [--bag BAG_NAME] [--host HOST] [--port PORT]
      録画開始（rosout は有効化しない）→ Ctrl+C で全停止。

  ros2 run r1_bringup remote_debug_client.py stop [--host HOST] [--port PORT]
      録画停止（one-shot）。

  ros2 run r1_bringup remote_debug_client.py rosout-on [--host HOST] [--port PORT]
      rosout 転送有効化（one-shot）。

  ros2 run r1_bringup remote_debug_client.py rosout-off [--host HOST] [--port PORT]
      rosout 転送無効化（one-shot）。

Example:
  # ローカルの rosbridge に接続してモニタリング
  ros2 run r1_bringup remote_debug_client.py monitor

  # miniPC（192.168.50.12）の rosbridge に接続してモニタリング
  ros2 run r1_bringup remote_debug_client.py monitor --host 192.168.50.12

  # rosout 有効化 + 録画開始（バッグ名・待機時間を指定）
  ros2 run r1_bringup remote_debug_client.py session --host 192.168.50.12 --bag 2026-05-17_match1 --delay 2.0

  # 録画のみ開始（rosout は有効化しない）
  ros2 run r1_bringup remote_debug_client.py start --host 192.168.50.12 --bag 2026-05-17_match1
"""

import argparse
import time

import roslibpy

_ROSOUT_LEVEL_NAMES = {10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}
_ROSOUT_LEVEL_COLORS = {
    10: "",  # DEBUG:  デフォルト
    20: "",  # INFO:   デフォルト
    30: "\033[33m",  # WARN:   yellow
    40: "\033[31m",  # ERROR:  red
    50: "\033[1;31m",  # FATAL:  bold red
}
_ANSI_RESET = "\033[0m"

_DEFAULT_HOST = "localhost"
_DEFAULT_PORT = 9090
_POST_PUBLISH_DELAY = 0.2  # publish 後にメッセージが届くまでの待機 [s]
_CONNECT_TIMEOUT = 10.0  # rosbridge 接続タイムアウト [s]


class RemoteDebugClient:
    def __init__(self, host: str, port: int) -> None:
        self._host = host
        self._port = port
        self._ros = roslibpy.Ros(host=host, port=port)
        self._record_start_pub = roslibpy.Topic(
            self._ros, "/record_start", "std_msgs/String"
        )
        self._record_stop_pub = roslibpy.Topic(
            self._ros, "/record_stop", "std_msgs/Empty"
        )
        self._enable_rosout_pub = roslibpy.Topic(
            self._ros, "/enable_publish_rosout", "std_msgs/Bool"
        )
        self._rosout_sub = roslibpy.Topic(
            self._ros, "/r1_rosout_bridge", "rcl_interfaces/Log"
        )

    def connect(self) -> None:
        print(f"rosbridge に接続中 ({self._host}:{self._port})...")
        self._ros.run(timeout=_CONNECT_TIMEOUT)
        self._rosout_sub.subscribe(self._on_rosout_bridge)
        print("接続しました")

    def disconnect(self) -> None:
        try:
            self._rosout_sub.unsubscribe()
        except Exception:
            pass
        self._ros.terminate()

    # ------------------------------------------------------------------
    # topic 操作
    # ------------------------------------------------------------------

    def _publish(self, topic: roslibpy.Topic, message: dict) -> None:
        topic.publish(roslibpy.Message(message))
        time.sleep(_POST_PUBLISH_DELAY)

    def start_recording(self, bag_name: str = "") -> None:
        self._publish(self._record_start_pub, {"data": bag_name})
        label = f'"{bag_name}"' if bag_name else "自動生成"
        print(f"録画開始 (bag: {label})")

    def stop_recording(self) -> None:
        self._publish(self._record_stop_pub, {})
        print("録画停止")

    def set_rosout(self, enable: bool) -> None:
        self._publish(self._enable_rosout_pub, {"data": enable})
        print(f"rosout 転送: {'有効' if enable else '無効'}")

    def _on_rosout_bridge(self, msg: dict) -> None:
        level_num = msg.get("level", 20)
        level = _ROSOUT_LEVEL_NAMES.get(level_num, f"LV{level_num}")
        color = _ROSOUT_LEVEL_COLORS.get(level_num, "")
        name = msg.get("name", "")
        text = msg.get("msg", "")
        print(f"{color}[{level}] [{name}]: {text}{_ANSI_RESET}", flush=True)

    # ------------------------------------------------------------------
    # コマンド実装
    # ------------------------------------------------------------------

    def run_monitor(self) -> None:
        """rosout 有効化 → 常駐（録画なし）。Ctrl+C で全停止。"""
        self.set_rosout(True)
        print("モニタリング中。Ctrl+C で停止します...")
        try:
            while self._ros.is_connected:
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            self.set_rosout(False)

    def run_session(self, bag_name: str, delay: float) -> None:
        """rosout 有効化 → 録画開始 → 常駐。Ctrl+C で全停止。"""
        self.set_rosout(True)
        print(f"{delay}s 後に録画を開始します...")
        time.sleep(delay)
        self.start_recording(bag_name)
        print("セッション実行中。Ctrl+C で停止します...")
        try:
            while self._ros.is_connected:
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            self.stop_recording()
            self.set_rosout(False)

    def run_start(self, bag_name: str) -> None:
        """録画開始（rosout は有効化しない）→ 常駐。Ctrl+C で全停止。"""
        self.start_recording(bag_name)
        print("録画中。Ctrl+C で停止します...")
        try:
            while self._ros.is_connected:
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            self.stop_recording()


def main() -> None:
    shared = argparse.ArgumentParser(add_help=False)
    shared.add_argument(
        "--host",
        default=_DEFAULT_HOST,
        metavar="HOST",
        help=f"rosbridge WebSocket ホスト（デフォルト: {_DEFAULT_HOST}）",
    )
    shared.add_argument(
        "--port",
        type=int,
        default=_DEFAULT_PORT,
        metavar="PORT",
        help=f"rosbridge WebSocket ポート（デフォルト: {_DEFAULT_PORT}）",
    )

    parser = argparse.ArgumentParser(
        description="remote_debug_node コントローラ（rosbridge WebSocket 経由）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        parents=[shared],
    )
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("monitor", parents=[shared], help="rosout 有効化 → 常駐（録画なし）")

    p_session = sub.add_parser("session", parents=[shared], help="rosout 有効化 → 録画開始 → 常駐")
    p_session.add_argument(
        "--bag", default="", metavar="BAG_NAME", help="バッグ名（省略で自動生成）"
    )
    p_session.add_argument(
        "--delay",
        type=float,
        default=1.0,
        metavar="SEC",
        help="rosout 有効化から録画開始までの待機時間 [s]（デフォルト: 1.0）",
    )

    p_start = sub.add_parser("start", parents=[shared], help="録画開始（rosout は有効化しない）→ 常駐")
    p_start.add_argument(
        "--bag", default="", metavar="BAG_NAME", help="バッグ名（省略で自動生成）"
    )

    sub.add_parser("stop", parents=[shared], help="録画停止（one-shot）")
    sub.add_parser("rosout-on", parents=[shared], help="rosout 転送有効化（one-shot）")
    sub.add_parser("rosout-off", parents=[shared], help="rosout 転送無効化（one-shot）")

    args = parser.parse_args()

    client = RemoteDebugClient(args.host, args.port)
    try:
        client.connect()
        if args.command == "monitor":
            client.run_monitor()
        elif args.command == "session":
            client.run_session(args.bag, args.delay)
        elif args.command == "start":
            client.run_start(args.bag)
        elif args.command == "stop":
            client.stop_recording()
        elif args.command == "rosout-on":
            client.set_rosout(True)
        elif args.command == "rosout-off":
            client.set_rosout(False)
    except roslibpy.core.RosTimeoutError:
        print(f"接続タイムアウト: {args.host}:{args.port} に接続できませんでした")
        raise SystemExit(1)
    finally:
        client.disconnect()


if __name__ == "__main__":
    main()
