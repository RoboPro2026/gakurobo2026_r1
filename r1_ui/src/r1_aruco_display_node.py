#!/usr/bin/env python3

from pathlib import Path
import os
import signal
import sys
from typing import Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from PyQt6.QtCore import QRect, QTimer, Qt
from PyQt6.QtGui import QPixmap, QTransform
from PyQt6.QtWidgets import QApplication, QLabel, QMainWindow, QVBoxLayout, QWidget
from rclpy.node import Node
from std_msgs.msg import Int32


def has_display_environment() -> bool:
    return "DISPLAY" in os.environ or "WAYLAND_DISPLAY" in os.environ


class ArucoDisplayWindow(QMainWindow):
    def __init__(
        self,
        marker_id: int,
        marker_image_path: Path,
        window_title: str,
        image_rotation_degrees: int,
        marker_geometry: Optional[QRect],
    ) -> None:
        super().__init__()
        self.current_marker_id = marker_id
        self.current_marker_image_path = marker_image_path
        self.image_rotation_degrees = image_rotation_degrees % 360
        self.marker_geometry = marker_geometry
        self._base_pixmap: Optional[QPixmap] = None

        self.setWindowTitle(window_title)
        self.resize(600, 680)

        central_widget = QWidget(self)

        self.marker_label = QLabel(central_widget)
        self.marker_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.marker_label.setMinimumSize(320, 320)

        self.info_label = QLabel(central_widget)
        self.info_label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.setCentralWidget(central_widget)

        if self.marker_geometry is None:
            layout = QVBoxLayout(central_widget)
            layout.addWidget(self.marker_label, stretch=1)
            layout.addWidget(self.info_label)
        else:
            self.marker_label.setMinimumSize(1, 1)
            self._apply_marker_geometry()

        self.update_marker(marker_id, marker_image_path)

    def update_marker(self, marker_id: int, marker_image_path: Path) -> None:
        self.current_marker_id = marker_id
        self.current_marker_image_path = marker_image_path
        self._base_pixmap = QPixmap(str(marker_image_path))
        self._refresh_pixmap()
        self.info_label.setText(f"marker_id={marker_id} / {marker_image_path.name}")

    def resizeEvent(self, event) -> None:  # type: ignore[override]
        super().resizeEvent(event)
        self._apply_marker_geometry()
        self._refresh_pixmap()

    def _apply_marker_geometry(self) -> None:
        if self.marker_geometry is None:
            return
        self.marker_label.setGeometry(self.marker_geometry)
        central_rect = self.centralWidget().rect()
        self.info_label.setGeometry(
            0,
            max(0, central_rect.height() - 24),
            central_rect.width(),
            24,
        )

    def _refresh_pixmap(self) -> None:
        if self._base_pixmap is None:
            return
        transform = QTransform().rotate(self.image_rotation_degrees)
        rotated_pixmap = self._base_pixmap.transformed(
            transform,
            Qt.TransformationMode.SmoothTransformation,
        )
        scaled = rotated_pixmap.scaled(
            self.marker_label.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        self.marker_label.setPixmap(scaled)


class ArucoDisplayNode(Node):
    def __init__(self) -> None:
        super().__init__("r1_aruco_display_node")

        self.declare_parameter("topic_name", "aruco_marker_id")
        self.declare_parameter("window_title", "R1 ArUco Display")
        self.declare_parameter("initial_marker_id", 0)
        self.declare_parameter("fullscreen", False)
        self.declare_parameter("screen_name", "")
        self.declare_parameter("image_rotation_degrees", 0)
        self.declare_parameter("marker_x", -1)
        self.declare_parameter("marker_y", -1)
        self.declare_parameter("marker_width", -1)
        self.declare_parameter("marker_height", -1)
        self.declare_parameter("spin_rate_hz", 100.0)
        default_marker_dir = str(
            Path(get_package_share_directory("r1_ui")) / "aruco_marker"
        )
        self.declare_parameter("marker_image_dir", default_marker_dir)

        self.topic_name = self.get_parameter("topic_name").get_parameter_value().string_value
        self.window_title = self.get_parameter("window_title").get_parameter_value().string_value
        self.current_marker_id = (
            self.get_parameter("initial_marker_id").get_parameter_value().integer_value
        )
        self.fullscreen = self.get_parameter("fullscreen").get_parameter_value().bool_value
        self.screen_name = self.get_parameter("screen_name").get_parameter_value().string_value
        self.image_rotation_degrees = (
            self.get_parameter("image_rotation_degrees").get_parameter_value().integer_value
        )
        self.marker_geometry = self._get_marker_geometry()
        self.spin_rate_hz = (
            self.get_parameter("spin_rate_hz").get_parameter_value().double_value
        )
        self.marker_image_dir = Path(
            self.get_parameter("marker_image_dir").get_parameter_value().string_value
        )

        if self.spin_rate_hz <= 0.0:
            raise ValueError("spin_rate_hz must be greater than zero")

        initial_marker_path = self._marker_image_path(self.current_marker_id)

        self.window = ArucoDisplayWindow(
            marker_id=self.current_marker_id,
            marker_image_path=initial_marker_path,
            window_title=self.window_title,
            image_rotation_degrees=self.image_rotation_degrees,
            marker_geometry=self.marker_geometry,
        )
        self._move_window_to_screen()
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
            f"Displaying marker image {initial_marker_path.name} "
            f"with image_rotation_degrees={self.image_rotation_degrees % 360} "
            f"and marker_geometry={self._marker_geometry_text()} "
            f"and subscribing to '{self.topic_name}' with spin_rate_hz={self.spin_rate_hz}."
        )

    def _get_marker_geometry(self) -> Optional[QRect]:
        marker_x = self.get_parameter("marker_x").get_parameter_value().integer_value
        marker_y = self.get_parameter("marker_y").get_parameter_value().integer_value
        marker_width = (
            self.get_parameter("marker_width").get_parameter_value().integer_value
        )
        marker_height = (
            self.get_parameter("marker_height").get_parameter_value().integer_value
        )
        geometry_values = (marker_x, marker_y, marker_width, marker_height)
        if all(value == -1 for value in geometry_values):
            return None
        if marker_x < 0 or marker_y < 0:
            raise ValueError(
                "marker_x and marker_y must be non-negative when marker geometry is specified"
            )
        if marker_width <= 0 or marker_height <= 0:
            raise ValueError(
                "marker_width and marker_height must be greater than zero "
                "when marker geometry is specified"
            )
        return QRect(marker_x, marker_y, marker_width, marker_height)

    def _marker_geometry_text(self) -> str:
        if self.marker_geometry is None:
            return "default"
        return (
            f"x={self.marker_geometry.x()}, y={self.marker_geometry.y()}, "
            f"width={self.marker_geometry.width()}, "
            f"height={self.marker_geometry.height()}"
        )

    def _move_window_to_screen(self) -> None:
        if self.screen_name == "":
            return

        screens = QApplication.screens()
        for screen in screens:
            if screen.name() == self.screen_name:
                self.window.winId()
                window_handle = self.window.windowHandle()
                if window_handle is not None:
                    window_handle.setScreen(screen)
                if self.fullscreen:
                    self.window.setGeometry(screen.geometry())
                else:
                    screen_geometry = screen.availableGeometry()
                    window_geometry = self.window.geometry()
                    self.window.move(
                        screen_geometry.x()
                        + (screen_geometry.width() - window_geometry.width()) // 2,
                        screen_geometry.y()
                        + (screen_geometry.height() - window_geometry.height()) // 2,
                    )
                self.get_logger().info(
                    f"Using screen '{screen.name()}' with geometry {screen.geometry()}."
                )
                return

        available_screen_names = ", ".join(screen.name() for screen in screens)
        self.get_logger().warning(
            f"screen_name '{self.screen_name}' was not found. "
            f"Available screens: {available_screen_names}"
        )

    def _marker_id_callback(self, msg: Int32) -> None:
        marker_id = int(msg.data)
        try:
            marker_image_path = self._marker_image_path(marker_id)
        except ValueError as exc:
            self.get_logger().warning(str(exc))
            return

        if marker_id == self.current_marker_id:
            return

        self.current_marker_id = marker_id
        self.window.update_marker(marker_id, marker_image_path)
        self.get_logger().info(f"Switched marker image to {marker_image_path.name}")

    def _marker_image_path(self, marker_id: int) -> Path:
        if marker_id < 0:
            raise ValueError("marker_id must be non-negative")

        marker_image_path = self.marker_image_dir / f"marker_{marker_id}.png"
        if not marker_image_path.is_file():
            raise ValueError(f"Marker image not found: {marker_image_path}")
        return marker_image_path


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)

    if not has_display_environment():
        print("No display server environment found.", file=sys.stderr)
        print("Run this node from a logged-in desktop terminal.", file=sys.stderr)
        print("Expected DISPLAY or WAYLAND_DISPLAY to be set.", file=sys.stderr)
        if rclpy.ok():
            rclpy.shutdown()
        raise SystemExit(1)

    # Wayland セッションでは Qt が xcb を選ぶと libxcb-cursor0 不足で落ちるため、
    # 明示指定がない場合だけ Wayland backend を優先する。
    if "QT_QPA_PLATFORM" not in os.environ and "WAYLAND_DISPLAY" in os.environ:
        os.environ["QT_QPA_PLATFORM"] = "wayland"

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
