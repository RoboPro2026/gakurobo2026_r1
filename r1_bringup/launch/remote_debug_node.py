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
    /r1_rosout_bridge (rcl_interfaces/Log): 転送先。rosbridge 経由で MSI Claw から購読する。
                                            200ms ごとにバッファをまとめて転送（負荷軽減）。
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

# record.bash と同じ CAN 系 topic 除外 regex
_CAN_TOPIC_REGEX = r'(^|/)(sabacan_[^/]*|(from|to)_can_bus[^/]*)$'

# rosout バッファのフラッシュ間隔
_ROSOUT_FLUSH_INTERVAL = 0.2  # [s]


class RemoteDebugNode(Node):
    def __init__(self) -> None:
        super().__init__("remote_debug_node")

        self._record_proc: Optional[subprocess.Popen] = None
        self._enable_publish_rosout: bool = False
        self._rosout_buffer: List[RosoutLog] = []

        # ----- rosbag recording -----
        self.create_subscription(String, "/record_start", self._on_record_start, 10)
        self.create_subscription(Empty, "/record_stop", self._on_record_stop, 10)

        # ----- rosout bridge -----
        self._rosout_bridge_pub = self.create_publisher(
            RosoutLog, "/r1_rosout_bridge", QoSProfile(depth=50)
        )
        self.create_subscription(
            Bool, "/enable_publish_rosout", self._on_enable_publish_rosout, 10
        )
        self.create_subscription(
            RosoutLog, "/rosout", self._on_rosout, QoSProfile(depth=50)
        )
        self.create_timer(_ROSOUT_FLUSH_INTERVAL, self._flush_rosout_buffer)

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

        self._record_proc = subprocess.Popen(cmd, cwd=_WORKSPACE_DIR)
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
        # SIGINT で ros2 bag record を正常終了させる
        self._record_proc.send_signal(signal.SIGINT)
        try:
            self._record_proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            self._record_proc.kill()
            self._record_proc.wait()
        self._record_proc = None
        self.get_logger().info("Recording stopped")

    # ------------------------------------------------------------------
    # rosout bridge
    # ------------------------------------------------------------------

    def _on_enable_publish_rosout(self, msg: Bool) -> None:
        self._enable_publish_rosout = msg.data
        self.get_logger().info(
            f"enable_publish_rosout: {'true' if msg.data else 'false'}"
        )

    def _on_rosout(self, msg: RosoutLog) -> None:
        if self._enable_publish_rosout:
            self._rosout_buffer.append(msg)

    def _flush_rosout_buffer(self) -> None:
        if not self._rosout_buffer:
            return
        for msg in self._rosout_buffer:
            self._rosout_bridge_pub.publish(msg)
        self._rosout_buffer.clear()

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
