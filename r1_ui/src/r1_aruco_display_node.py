#!/usr/bin/env python3

import signal
import sys
from typing import Optional

import cv2
import numpy as np
import rclpy
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtWidgets import QApplication, QLabel, QMainWindow, QVBoxLayout, QWidget
from rclpy.node import Node
from std_msgs.msg import Int32


class ArucoDisplayWindow(QMainWindow):
    def __init__(
        self,
        marker_id: int,
        dictionary_name: str,
        marker_size_px: int,
        window_title: str,
    ) -> None:
        super().__init__()
        self.marker_size_px = marker_size_px
        self.current_marker_id = marker_id
        self.dictionary_name = dictionary_name
        self._base_pixmap: Optional[QPixmap] = None

        self.setWindowTitle(window_title)
        self.resize(max(marker_size_px, 480), max(marker_size_px + 80, 560))

        central_widget = QWidget(self)
        layout = QVBoxLayout(central_widget)

        self.marker_label = QLabel()
        self.marker_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.marker_label.setMinimumSize(320, 320)

        self.info_label = QLabel()
        self.info_label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        layout.addWidget(self.marker_label, stretch=1)
        layout.addWidget(self.info_label)
        self.setCentralWidget(central_widget)

        self.update_marker(marker_id, dictionary_name)

    def update_marker(self, marker_id: int, dictionary_name: str) -> None:
        self.current_marker_id = marker_id
        self.dictionary_name = dictionary_name
        marker = self._render_marker(dictionary_name, marker_id, self.marker_size_px)
        qimage = QImage(
            marker.data,
            marker.shape[1],
            marker.shape[0],
            marker.strides[0],
            QImage.Format.Format_Grayscale8,
        )
        self._base_pixmap = QPixmap.fromImage(qimage.copy())
        self._refresh_pixmap()
        self.info_label.setText(f"{dictionary_name} / marker_id={marker_id}")

    def resizeEvent(self, event) -> None:  # type: ignore[override]
        super().resizeEvent(event)
        self._refresh_pixmap()

    def _refresh_pixmap(self) -> None:
        if self._base_pixmap is None:
            return
        scaled = self._base_pixmap.scaled(
            self.marker_label.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        self.marker_label.setPixmap(scaled)

    @staticmethod
    def _render_marker(dictionary_name: str, marker_id: int, marker_size_px: int) -> np.ndarray:
        dictionary = ArucoDisplayNode.make_dictionary(dictionary_name)
        if hasattr(cv2.aruco, "generateImageMarker"):
            return cv2.aruco.generateImageMarker(dictionary, marker_id, marker_size_px)

        marker = np.full((marker_size_px, marker_size_px), 255, dtype=np.uint8)
        cv2.aruco.drawMarker(dictionary, marker_id, marker_size_px, marker, 1)
        return marker


class ArucoDisplayNode(Node):
    def __init__(self) -> None:
        super().__init__("r1_aruco_display_node")

        self.declare_parameter("topic_name", "aruco_marker_id")
        self.declare_parameter("window_title", "R1 ArUco Display")
        self.declare_parameter("dictionary", "DICT_4X4_50")
        self.declare_parameter("initial_marker_id", 0)
        self.declare_parameter("marker_size_px", 600)
        self.declare_parameter("fullscreen", False)
        self.declare_parameter("spin_rate_hz", 100.0)

        self.topic_name = self.get_parameter("topic_name").get_parameter_value().string_value
        self.window_title = self.get_parameter("window_title").get_parameter_value().string_value
        self.dictionary_name = self.get_parameter("dictionary").get_parameter_value().string_value
        self.marker_size_px = self.get_parameter("marker_size_px").get_parameter_value().integer_value
        self.current_marker_id = (
            self.get_parameter("initial_marker_id").get_parameter_value().integer_value
        )
        self.fullscreen = self.get_parameter("fullscreen").get_parameter_value().bool_value
        self.spin_rate_hz = (
            self.get_parameter("spin_rate_hz").get_parameter_value().double_value
        )

        if self.marker_size_px <= 0:
            raise ValueError("marker_size_px must be greater than zero")
        if self.spin_rate_hz <= 0.0:
            raise ValueError("spin_rate_hz must be greater than zero")

        self._validate_marker(self.dictionary_name, self.current_marker_id)

        self.window = ArucoDisplayWindow(
            marker_id=self.current_marker_id,
            dictionary_name=self.dictionary_name,
            marker_size_px=self.marker_size_px,
            window_title=self.window_title,
        )
        if self.fullscreen:
            self.window.showFullScreen()
        else:
            self.window.show()

        self.subscription = self.create_subscription(
            Int32,
            self.topic_name,
            self._marker_id_callback,
            10,
        )

        self.get_logger().info(
            f"Displaying {self.dictionary_name} marker {self.current_marker_id} "
            f"and subscribing to '{self.topic_name}' with spin_rate_hz={self.spin_rate_hz}."
        )

    @staticmethod
    def make_dictionary(dictionary_name: str):
        dictionary_id = getattr(cv2.aruco, dictionary_name, None)
        if dictionary_id is None:
            raise ValueError(f"Unsupported ArUco dictionary: {dictionary_name}")
        return cv2.aruco.getPredefinedDictionary(dictionary_id)

    def _marker_id_callback(self, msg: Int32) -> None:
        marker_id = int(msg.data)
        try:
            self._validate_marker(self.dictionary_name, marker_id)
        except ValueError as exc:
            self.get_logger().warning(str(exc))
            return

        if marker_id == self.current_marker_id:
            return

        self.current_marker_id = marker_id
        self.window.update_marker(marker_id, self.dictionary_name)
        self.get_logger().info(f"Switched marker to {marker_id}")

    def _validate_marker(self, dictionary_name: str, marker_id: int) -> None:
        dictionary = self.make_dictionary(dictionary_name)
        marker_count = int(dictionary.bytesList.shape[0])
        if marker_id < 0 or marker_id >= marker_count:
            raise ValueError(
                f"marker_id={marker_id} is outside the valid range 0..{marker_count - 1} "
                f"for {dictionary_name}"
            )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)

    app = QApplication.instance() or QApplication(sys.argv[:1])
    node = ArucoDisplayNode()

    def spin_ros_once() -> None:
        if not rclpy.ok():
            return
        try:
            rclpy.spin_once(node, timeout_sec=0.0)
        except Exception as exc:
            if "context is not valid" in str(exc):
                return
            raise

    spin_timer = QTimer()
    spin_timer.timeout.connect(spin_ros_once)
    spin_timer.start(max(1, round(1000.0 / node.spin_rate_hz)))

    signal.signal(signal.SIGINT, lambda *_: app.quit())
    signal.signal(signal.SIGTERM, lambda *_: app.quit())

    exit_code = 0
    try:
        exit_code = app.exec()
    finally:
        spin_timer.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
