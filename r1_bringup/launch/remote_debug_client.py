#!/usr/bin/env python3
"""
remote_debug_node を制御するクライアント。
シェルスクリプトより確実にトピックを届けるため rclpy を使う。

session / start コマンドは rosbridge を起動して常駐し、Ctrl+C で全停止する。
stop / rosout-on / rosout-off は one-shot で終了する。

Usage:
  ros2 run r1_bringup remote_debug_client.py session [--bag BAG_NAME] [--delay SEC] [--rosbridge-delay SEC]
      rosbridge 起動 → rosout 有効化 → --delay 秒待機 → 録画開始 → Ctrl+C で全停止。

  ros2 run r1_bringup remote_debug_client.py start [--bag BAG_NAME] [--rosbridge-delay SEC]
      rosbridge 起動 → 録画開始（rosout は有効化しない）→ Ctrl+C で全停止。

  ros2 run r1_bringup remote_debug_client.py stop
      録画停止（one-shot、rosbridge は起動しない）。

  ros2 run r1_bringup remote_debug_client.py rosout-on
      rosout 転送有効化（one-shot、rosbridge は起動しない）。

  ros2 run r1_bringup remote_debug_client.py rosout-off
      rosout 転送無効化（one-shot、rosbridge は起動しない）。

Example:
  # rosout 有効化 + 録画開始（バッグ名自動生成、1秒待機）
  ros2 run r1_bringup remote_debug_client.py session

  # rosout 有効化 + 録画開始（バッグ名・待機時間を指定）
  ros2 run r1_bringup remote_debug_client.py session --bag 2026-05-17_match1 --delay 2.0

  # 録画のみ開始（rosout は有効化しない）
  ros2 run r1_bringup remote_debug_client.py start --bag 2026-05-17_match1
"""

import argparse
import signal
import subprocess
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rcl_interfaces.msg import Log as RosoutLog
from std_msgs.msg import Bool, Empty, String

_ROSOUT_LEVEL_NAMES = {10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}

_SUBSCRIBER_WAIT_TIMEOUT = 5.0   # サブスクライバー待機のタイムアウト [s]
_POST_PUBLISH_DELAY = 0.2        # publish 後にメッセージが届くまでの待機 [s]
_DEFAULT_ROSBRIDGE_DELAY = 1.0   # rosbridge 起動後の待機時間 [s]


class RemoteDebugClient(Node):
    def __init__(self) -> None:
        super().__init__("remote_debug_client")
        self._record_start_pub = self.create_publisher(String, "/record_start", 10)
        self._record_stop_pub = self.create_publisher(Empty, "/record_stop", 10)
        self._enable_rosout_pub = self.create_publisher(Bool, "/enable_publish_rosout", 10)
        self._rosbridge_proc: Optional[subprocess.Popen] = None

        self.create_subscription(
            RosoutLog, "/r1_rosout_bridge", self._on_rosout_bridge, QoSProfile(depth=50)
        )

    # ------------------------------------------------------------------
    # rosbridge
    # ------------------------------------------------------------------

    def start_rosbridge(self, delay: float = _DEFAULT_ROSBRIDGE_DELAY) -> None:
        self.get_logger().info("rosbridge 起動中...")
        self._rosbridge_proc = subprocess.Popen(
            ["ros2", "launch", "rosbridge_server", "rosbridge_websocket_launch.xml"]
        )
        self.get_logger().info(f"rosbridge 待機中 ({delay}s)...")
        time.sleep(delay)

    def stop_rosbridge(self) -> None:
        if self._rosbridge_proc is None or self._rosbridge_proc.poll() is not None:
            return
        self._rosbridge_proc.send_signal(signal.SIGINT)
        try:
            self._rosbridge_proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            self._rosbridge_proc.kill()
            self._rosbridge_proc.wait()
        self._rosbridge_proc = None
        self.get_logger().info("rosbridge 停止")

    # ------------------------------------------------------------------
    # topic 操作
    # ------------------------------------------------------------------

    def _wait_for_subscriber(self, publisher, timeout: float = _SUBSCRIBER_WAIT_TIMEOUT) -> bool:
        """サブスクライバーが DDS に見つかるまで待機する。"""
        deadline = time.time() + timeout
        while publisher.get_subscription_count() == 0:
            if time.time() >= deadline:
                self.get_logger().warn(
                    f"{publisher.topic_name}: サブスクライバーが見つかりません "
                    f"(timeout={timeout}s) — remote_debug_node が起動しているか確認してください"
                )
                return False
            time.sleep(0.05)
        return True

    def _publish(self, publisher, msg) -> None:
        self._wait_for_subscriber(publisher)
        publisher.publish(msg)
        time.sleep(_POST_PUBLISH_DELAY)

    def start_recording(self, bag_name: str = "") -> None:
        msg = String()
        msg.data = bag_name
        self._publish(self._record_start_pub, msg)
        label = f'"{bag_name}"' if bag_name else "自動生成"
        self.get_logger().info(f"録画開始 (bag: {label})")

    def stop_recording(self) -> None:
        self._publish(self._record_stop_pub, Empty())
        self.get_logger().info("録画停止")

    def set_rosout(self, enable: bool) -> None:
        msg = Bool()
        msg.data = enable
        self._publish(self._enable_rosout_pub, msg)
        self.get_logger().info(f"rosout 転送: {'有効' if enable else '無効'}")

    def _on_rosout_bridge(self, msg: RosoutLog) -> None:
        level = _ROSOUT_LEVEL_NAMES.get(msg.level, f"LV{msg.level}")
        print(f"[{level}] [{msg.name}]: {msg.msg}", flush=True)

    # ------------------------------------------------------------------
    # コマンド実装
    # ------------------------------------------------------------------

    def run_session(self, bag_name: str, delay: float, rosbridge_delay: float) -> None:
        """rosout 有効化 → 録画開始 → 常駐。Ctrl+C で全停止。"""
        self.start_rosbridge(rosbridge_delay)
        self.set_rosout(True)
        self.get_logger().info(f"{delay}s 後に録画を開始します...")
        time.sleep(delay)
        self.start_recording(bag_name)
        self.get_logger().info("セッション実行中。Ctrl+C で停止します...")
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
        except KeyboardInterrupt:
            pass
        finally:
            self.stop_recording()
            self.set_rosout(False)
            self.stop_rosbridge()

    def run_start(self, bag_name: str, rosbridge_delay: float) -> None:
        """録画開始（rosout は有効化しない）→ 常駐。Ctrl+C で全停止。"""
        self.start_rosbridge(rosbridge_delay)
        self.start_recording(bag_name)
        self.get_logger().info("録画中。Ctrl+C で停止します...")
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
        except KeyboardInterrupt:
            pass
        finally:
            self.stop_recording()
            self.stop_rosbridge()

    # ------------------------------------------------------------------
    # lifecycle
    # ------------------------------------------------------------------

    def destroy_node(self) -> None:
        self.stop_rosbridge()
        super().destroy_node()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="remote_debug_node コントローラ",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    sub = parser.add_subparsers(dest="command", required=True)

    p_session = sub.add_parser("session", help="rosout 有効化 → 録画開始 → 常駐")
    p_session.add_argument("--bag", default="", metavar="BAG_NAME", help="バッグ名（省略で自動生成）")
    p_session.add_argument(
        "--delay", type=float, default=1.0, metavar="SEC",
        help="rosout 有効化から録画開始までの待機時間 [s]（デフォルト: 1.0）",
    )
    p_session.add_argument(
        "--rosbridge-delay", type=float, default=_DEFAULT_ROSBRIDGE_DELAY, metavar="SEC",
        help=f"rosbridge 起動後の待機時間 [s]（デフォルト: {_DEFAULT_ROSBRIDGE_DELAY}）",
    )

    p_start = sub.add_parser("start", help="録画開始（rosout は有効化しない）→ 常駐")
    p_start.add_argument("--bag", default="", metavar="BAG_NAME", help="バッグ名（省略で自動生成）")
    p_start.add_argument(
        "--rosbridge-delay", type=float, default=_DEFAULT_ROSBRIDGE_DELAY, metavar="SEC",
        help=f"rosbridge 起動後の待機時間 [s]（デフォルト: {_DEFAULT_ROSBRIDGE_DELAY}）",
    )

    sub.add_parser("stop", help="録画停止（one-shot）")
    sub.add_parser("rosout-on", help="rosout 転送有効化（one-shot）")
    sub.add_parser("rosout-off", help="rosout 転送無効化（one-shot）")

    args = parser.parse_args()

    rclpy.init()
    client = RemoteDebugClient()

    try:
        if args.command == "session":
            client.run_session(args.bag, args.delay, args.rosbridge_delay)
        elif args.command == "start":
            client.run_start(args.bag, args.rosbridge_delay)
        elif args.command == "stop":
            client.stop_recording()
        elif args.command == "rosout-on":
            client.set_rosout(True)
        elif args.command == "rosout-off":
            client.set_rosout(False)
    finally:
        client.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
