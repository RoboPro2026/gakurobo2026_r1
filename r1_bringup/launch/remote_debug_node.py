#!/usr/bin/env python3
"""
遠隔デバッグ用ノード。
rosbridge 経由で MSI Claw から各種デバッグ機能を制御する。

[rosbag recording]
  Subscribe:
    /record_start (std_msgs/String) : data にバッグ名を指定して録画開始。空文字なら自動名。
    /record_stop  (std_msgs/Empty)  : 録画停止。

[rosout bridge]
  Subscribe:
    /enable_publish_rosout (std_msgs/Bool): true で /rosout の転送を開始、false で停止。
    /rosout (rcl_interfaces/Log)         : ROS ログ集約トピック。
  Publish:
    /r1_rosout_bridge_text (std_msgs/String): 転送先。rosbridge 経由で MSI Claw から購読する。
"""

import os
import signal
import subprocess
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rcl_interfaces.msg import Log as RosoutLog
from std_msgs.msg import Bool, Empty, String

_WORKSPACE_DIR = os.path.expanduser("~/ros2_ws")

# record.bash と同じ CAN 系 topic 除外 regex。
# ros2 bag の --exclude は topic 名全体に対して評価されるため、
# "/sabacan_robomas_ref8/motor2" のような子 topic も含めて、
# 名前のどこかに sabacan/from_can_bus/to_can_bus を含む topic を除外する。
_CAN_TOPIC_REGEX = r'.*(sabacan|from_can_bus|to_can_bus).*'

_ROSOUT_QOS_DEPTH = 1000
_ROSOUT_TEXT_FLUSH_INTERVAL = 0.1  # [s]
_ROSOUT_LEVEL_NAMES = {10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}
_ROSOUT_LEVEL_COLORS = {
    10: "",  # DEBUG: default
    20: "",  # INFO: default
    30: "\033[33m",  # WARN: yellow
    40: "\033[31m",  # ERROR: red
    50: "\033[1;31m",  # FATAL: bold red
}
_ANSI_RESET = "\033[0m"

# ros2 bag は終了時に metadata.yaml を書き出すため、停止後に少し待つ。
_RECORD_STOP_TIMEOUT = 5.0  # [s]


class RemoteDebugNode(Node):
    def __init__(self) -> None:
        super().__init__("remote_debug_node")

        self._record_proc: Optional[subprocess.Popen] = None
        self._enable_publish_rosout: bool = False
        self._rosout_text_buffer: List[str] = []

        # ----- rosbag recording -----
        self.create_subscription(String, "/record_start", self._on_record_start, 10)
        self.create_subscription(Empty, "/record_stop", self._on_record_stop, 10)

        # ----- rosout bridge -----
        self._rosout_bridge_text_pub = self.create_publisher(
            String, "/r1_rosout_bridge_text", QoSProfile(depth=_ROSOUT_QOS_DEPTH)
        )
        self.create_subscription(
            Bool, "/enable_publish_rosout", self._on_enable_publish_rosout, 10
        )
        self.create_subscription(
            RosoutLog, "/rosout", self._on_rosout, QoSProfile(depth=_ROSOUT_QOS_DEPTH)
        )
        self.create_timer(_ROSOUT_TEXT_FLUSH_INTERVAL, self._flush_rosout_text_buffer)

        self.get_logger().info("remote_debug_node started")

    # ------------------------------------------------------------------
    # rosbag recording
    # ------------------------------------------------------------------

    def _on_record_start(self, msg: String) -> None:
        # プロセスが自然終了していた場合は状態をリセット
        if self._record_proc is not None and self._record_proc.poll() is not None:
            self.get_logger().info("Recording process ended")
            self._record_proc = None

        if self._is_recording():
            self.get_logger().warn("Already recording, ignoring start request")
            return

        bag_name = msg.data.strip()
        cmd = ["ros2", "bag", "record", "-a", "--exclude", _CAN_TOPIC_REGEX]
        if bag_name:
            cmd += ["-o", bag_name]

        # ros2 CLI とその子プロセスをまとめて停止できるように、
        # 録画プロセスは専用の process group で起動する。
        self._record_proc = subprocess.Popen(
            cmd, cwd=_WORKSPACE_DIR, start_new_session=True
        )
        self.get_logger().info(
            f'Recording started (bag: "{bag_name if bag_name else "auto"}")'
        )

    def _on_record_stop(self, _msg: Empty) -> None:
        self._stop_recording()

    def _is_recording(self) -> bool:
        return self._record_proc is not None and self._record_proc.poll() is None

    def _stop_recording(self) -> None:
        if not self._is_recording():
            return
        proc = self._record_proc
        if proc is None:
            return

        # SIGINT で ros2 bag record を正常終了させる。process group 全体へ送ることで、
        # ros2 CLI 配下の recorder まで Ctrl+C 相当の停止要求を届ける。
        os.killpg(proc.pid, signal.SIGINT)
        try:
            return_code = proc.wait(timeout=_RECORD_STOP_TIMEOUT)
            self.get_logger().info(f"Recording process exited: {return_code}")
        except subprocess.TimeoutExpired:
            self.get_logger().error("Recording process did not stop, killing it")
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()
        self._record_proc = None
        self.get_logger().info("Recording stopped")

    # ------------------------------------------------------------------
    # rosout bridge
    # ------------------------------------------------------------------

    def _on_enable_publish_rosout(self, msg: Bool) -> None:
        if not msg.data:
            self._flush_rosout_text_buffer()
        self._enable_publish_rosout = msg.data
        self.get_logger().info(
            f"enable_publish_rosout: {'true' if msg.data else 'false'}"
        )

    def _on_rosout(self, msg: RosoutLog) -> None:
        if self._enable_publish_rosout:
            self._rosout_text_buffer.append(self._format_rosout(msg))

    def _flush_rosout_text_buffer(self) -> None:
        if not self._rosout_text_buffer:
            return
        msg = String()
        msg.data = "\n".join(self._rosout_text_buffer)
        self._rosout_text_buffer.clear()
        self._rosout_bridge_text_pub.publish(msg)

    def _format_rosout(self, msg: RosoutLog) -> str:
        level = _ROSOUT_LEVEL_NAMES.get(msg.level, f"LV{msg.level}")
        color = _ROSOUT_LEVEL_COLORS.get(msg.level, "")
        return f"{color}[{level}] [{msg.name}]: {msg.msg}{_ANSI_RESET}"

    # ------------------------------------------------------------------
    # lifecycle
    # ------------------------------------------------------------------

    def destroy_node(self) -> None:
        self._stop_recording()
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = RemoteDebugNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
