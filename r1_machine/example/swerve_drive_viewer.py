#!/usr/bin/env python3

import math
import time
from dataclasses import dataclass
from typing import Optional

import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.patches import Arc, FancyArrowPatch
from matplotlib.lines import Line2D
from matplotlib.widgets import Button, CheckButtons, Slider, TextBox
import rclpy
from geometry_msgs.msg import Twist
from r1_msgs.msg import SwerveDrive
from rclpy.node import Node
from sensor_msgs.msg import Imu


@dataclass
class SwerveState:
    omega: list[float]
    theta: list[float]
    stamp_monotonic: float


@dataclass
class CmdVelState:
    vx: float
    vy: float
    omega: float
    stamp_monotonic: float


@dataclass
class ImuYawState:
    yaw: float
    stamp_monotonic: float


class SwerveDriveViewer(Node):
    def __init__(self) -> None:
        super().__init__("swerve_drive_viewer")

        self.declare_parameter("topic", "/swerve_drive_ref")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("cmd_vel_pub_topic", "/cmd_vel")
        self.declare_parameter("imu_pub_topic", "/bno086/imu/data_raw")
        self.declare_parameter("imu_sub_topic", "/bno086/imu/data_raw")
        self.declare_parameter("rotate_robot", True)
        self.declare_parameter("wheel_radius", 0.1)
        self.declare_parameter("swerve_v_is_angular", True)
        self.declare_parameter("robot_length", 0.5)
        self.declare_parameter("robot_width", 0.5)
        self.declare_parameter("vector_scale", 0.3)
        self.declare_parameter("cmd_vel_vector_scale", 0.3)
        self.declare_parameter("omega_arc_radius_scale", 0.45)
        self.declare_parameter("show_legend", True)
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("always_on_top", False)
        self.declare_parameter("raise_window", False)
        self.declare_parameter("enable_cmd_vel_publish_ui", True)
        self.declare_parameter("cmd_vel_publish_rate_hz", 20.0)
        self.declare_parameter("cmd_vel_limit_vx", 2.0)
        self.declare_parameter("cmd_vel_limit_vy", 2.0)
        self.declare_parameter("cmd_vel_limit_omega", 6.28)
        self.declare_parameter("enable_imu_publish_ui", True)
        self.declare_parameter("imu_publish_rate_hz", 50.0)
        self.declare_parameter("imu_yaw_limit", 3.141592653589793)
        self.declare_parameter("imu_frame_id", "")

        self._topic = self.get_parameter("topic").get_parameter_value().string_value
        self._cmd_vel_topic = (
            self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        )
        self._cmd_vel_pub_topic = (
            self.get_parameter("cmd_vel_pub_topic").get_parameter_value().string_value
        )
        self._imu_pub_topic = (
            self.get_parameter("imu_pub_topic").get_parameter_value().string_value
        )
        self._imu_sub_topic = (
            self.get_parameter("imu_sub_topic").get_parameter_value().string_value
        )
        self._rotate_robot = (
            self.get_parameter("rotate_robot").get_parameter_value().bool_value
        )
        self._wheel_radius = (
            self.get_parameter("wheel_radius").get_parameter_value().double_value
        )
        self._swerve_v_is_angular = (
            self.get_parameter("swerve_v_is_angular").get_parameter_value().bool_value
        )
        self._robot_length = (
            self.get_parameter("robot_length").get_parameter_value().double_value
        )
        self._robot_width = (
            self.get_parameter("robot_width").get_parameter_value().double_value
        )
        self._vector_scale = (
            self.get_parameter("vector_scale").get_parameter_value().double_value
        )
        self._cmd_vel_vector_scale = (
            self.get_parameter("cmd_vel_vector_scale").get_parameter_value().double_value
        )
        self._omega_arc_radius_scale = (
            self.get_parameter("omega_arc_radius_scale").get_parameter_value().double_value
        )
        self._show_legend = (
            self.get_parameter("show_legend").get_parameter_value().bool_value
        )
        self._rate_hz = self.get_parameter("rate_hz").get_parameter_value().double_value
        self._always_on_top = (
            self.get_parameter("always_on_top").get_parameter_value().bool_value
        )
        self._raise_window = (
            self.get_parameter("raise_window").get_parameter_value().bool_value
        )
        self._enable_cmd_vel_publish_ui = (
            self.get_parameter("enable_cmd_vel_publish_ui")
            .get_parameter_value()
            .bool_value
        )
        self._cmd_vel_publish_rate_hz = (
            self.get_parameter("cmd_vel_publish_rate_hz")
            .get_parameter_value()
            .double_value
        )
        self._cmd_vel_limit_vx = (
            self.get_parameter("cmd_vel_limit_vx").get_parameter_value().double_value
        )
        self._cmd_vel_limit_vy = (
            self.get_parameter("cmd_vel_limit_vy").get_parameter_value().double_value
        )
        self._cmd_vel_limit_omega = (
            self.get_parameter("cmd_vel_limit_omega").get_parameter_value().double_value
        )
        self._enable_imu_publish_ui = (
            self.get_parameter("enable_imu_publish_ui")
            .get_parameter_value()
            .bool_value
        )
        self._imu_publish_rate_hz = (
            self.get_parameter("imu_publish_rate_hz").get_parameter_value().double_value
        )
        self._imu_yaw_limit = (
            self.get_parameter("imu_yaw_limit").get_parameter_value().double_value
        )
        self._imu_frame_id = (
            self.get_parameter("imu_frame_id").get_parameter_value().string_value
        )
        if self._rate_hz <= 0.0:
            self._rate_hz = 30.0
        if self._cmd_vel_publish_rate_hz <= 0.0:
            self._cmd_vel_publish_rate_hz = 20.0
        if self._imu_publish_rate_hz <= 0.0:
            self._imu_publish_rate_hz = 50.0
        if self._cmd_vel_vector_scale <= 0.0:
            self._cmd_vel_vector_scale = self._vector_scale
        if self._omega_arc_radius_scale <= 0.0:
            self._omega_arc_radius_scale = 0.45
        if self._wheel_radius <= 0.0:
            self.get_logger().warn("wheel_radius <= 0.0, treating swerve speed as already-linear.")
            self._swerve_v_is_angular = False

        mpl.rcParams["figure.raise_window"] = bool(self._raise_window)

        self._latest: Optional[SwerveState] = None
        self._cmd_vel: Optional[CmdVelState] = None
        self._imu_yaw: Optional[ImuYawState] = None
        self._heading_est = 0.0
        self._heading_last_monotonic: Optional[float] = None
        self._cmd_vel_pub = None
        self._cmd_vel_auto_pub_enabled = False
        self._cmd_vel_auto_pub_last_monotonic: Optional[float] = None
        self._ui_vx = 0.0
        self._ui_vy = 0.0
        self._ui_omega = 0.0
        self._imu_pub = None
        self._imu_auto_pub_enabled = False
        self._imu_auto_pub_last_monotonic: Optional[float] = None
        self._ui_imu_yaw = 0.0

        self.create_subscription(SwerveDrive, self._topic, self._on_msg, 10)
        self.create_subscription(Twist, self._cmd_vel_topic, self._on_cmd_vel, 10)
        self.create_subscription(Imu, self._imu_sub_topic, self._on_imu, 10)
        if self._enable_cmd_vel_publish_ui:
            self._cmd_vel_pub = self.create_publisher(
                Twist, self._cmd_vel_pub_topic, 10
            )
        if self._enable_imu_publish_ui:
            self._imu_pub = self.create_publisher(Imu, self._imu_pub_topic, 10)
        self.get_logger().info(f"Subscribing: {self._topic}")
        self.get_logger().info(f"Subscribing: {self._cmd_vel_topic}")
        self.get_logger().info(f"Subscribing: {self._imu_sub_topic}")
        if self._enable_cmd_vel_publish_ui:
            self.get_logger().info(f"Publishing (UI): {self._cmd_vel_pub_topic}")
        if self._enable_imu_publish_ui:
            self.get_logger().info(f"Publishing (UI): {self._imu_pub_topic}")
        self.get_logger().info(
            f"matplotlib backend: {mpl.get_backend()}, raise_window={self._raise_window}, always_on_top={self._always_on_top}"
        )

        self._init_plot()

    def _qt_stays_on_top_hint(self, qt) -> Optional[object]:
        if hasattr(qt, "WindowStaysOnTopHint"):
            return qt.WindowStaysOnTopHint
        window_type = getattr(qt, "WindowType", None)
        if window_type is not None and hasattr(window_type, "WindowStaysOnTopHint"):
            return window_type.WindowStaysOnTopHint
        return None

    def _set_always_on_top(self, enable: bool) -> None:
        manager = getattr(self._fig.canvas, "manager", None)
        window = getattr(manager, "window", None)
        if window is None:
            return

        # TkAgg backend
        for method_name in ("wm_attributes", "attributes"):
            method = getattr(window, method_name, None)
            if method is None:
                continue
            try:
                method("-topmost", 1 if enable else 0)
                return
            except Exception:
                pass

        # Qt backend (PyQt / PySide)
        if hasattr(window, "setWindowFlag") or hasattr(window, "setWindowFlags"):
            qt = None
            for module in (
                "PySide6.QtCore",
                "PyQt6.QtCore",
                "PySide2.QtCore",
                "PyQt5.QtCore",
            ):
                try:
                    qt = __import__(module, fromlist=["Qt"]).Qt
                    break
                except Exception:
                    continue
            if qt is None:
                return
            hint = self._qt_stays_on_top_hint(qt)
            if hint is None:
                return
            try:
                if hasattr(window, "windowFlags") and hasattr(window, "setWindowFlags"):
                    flags = window.windowFlags()
                    new_flags = (flags | hint) if enable else (flags & ~hint)
                    window.setWindowFlags(new_flags)
                else:
                    window.setWindowFlag(hint, enable)
                window.show()  # apply flags (may raise once depending on backend/WM)
            except Exception:
                pass
            return

        # GTK backend
        if hasattr(window, "set_keep_above"):
            try:
                window.set_keep_above(bool(enable))
            except Exception:
                pass
            return

    def _on_cmd_vel(self, msg: Twist) -> None:
        now = time.monotonic()
        self._cmd_vel = CmdVelState(
            vx=float(msg.linear.x),
            vy=float(msg.linear.y),
            omega=float(msg.angular.z),
            stamp_monotonic=now,
        )

    def _quat_to_yaw(self, x: float, y: float, z: float, w: float) -> float:
        # yaw (Z axis rotation), ROS standard (ENU), from quaternion.
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _on_imu(self, msg: Imu) -> None:
        now = time.monotonic()
        q = msg.orientation
        yaw = self._quat_to_yaw(float(q.x), float(q.y), float(q.z), float(q.w))
        self._imu_yaw = ImuYawState(yaw=yaw, stamp_monotonic=now)

    def _on_msg(self, msg: SwerveDrive) -> None:
        self._latest = SwerveState(
            omega=[
                float(msg.omega0),
                float(msg.omega1),
                float(msg.omega2),
                float(msg.omega3),
            ],
            theta=[
                float(msg.theta0),
                float(msg.theta1),
                float(msg.theta2),
                float(msg.theta3),
            ],
            stamp_monotonic=time.monotonic(),
        )

    def _wheel_positions(self) -> tuple[list[float], list[float]]:
        half_l = 0.5 * self._robot_length
        half_w = 0.5 * self._robot_width

        x = [half_l, -half_l, -half_l, half_l]
        y = [half_w, half_w, -half_w, -half_w]
        return x, y

    def _rotate_xy(
        self, x: list[float], y: list[float], yaw: float
    ) -> tuple[list[float], list[float]]:
        c = math.cos(yaw)
        s = math.sin(yaw)
        xr: list[float] = []
        yr: list[float] = []
        for xi, yi in zip(x, y):
            xr.append(c * xi - s * yi)
            yr.append(s * xi + c * yi)
        return xr, yr

    def _clamp01(self, value: float) -> float:
        return 0.0 if value <= 0.0 else (1.0 if value >= 1.0 else value)

    def _swerve_speed_linear(self, omega_radps: float) -> float:
        if not self._swerve_v_is_angular:
            # legacy: treat incoming as already-linear
            return omega_radps
        return self._wheel_radius * omega_radps

    def _init_plot(self) -> None:
        plt.ion()
        self._fig = plt.figure(figsize=(12, 7))
        gs = self._fig.add_gridspec(1, 2, width_ratios=[3.6, 1.4], wspace=0.06)
        self._ax = self._fig.add_subplot(gs[0, 0])
        ui_gs = gs[0, 1].subgridspec(2, 1, height_ratios=[1.4, 3.6], hspace=0.05)
        self._ax_ui_status = self._fig.add_subplot(ui_gs[0, 0])
        self._ax_ui_controls = self._fig.add_subplot(ui_gs[1, 0])
        self._ax_ui_status.set_axis_off()
        self._ax_ui_controls.set_axis_off()

        self._fig.canvas.manager.set_window_title("swerve_drive_ref viewer")
        self._set_always_on_top(self._always_on_top)

        self._ax.set_aspect("equal", adjustable="box")
        self._ax.set_xlabel("x (Twist.linear.x)")
        self._ax.set_ylabel("y (Twist.linear.y)")
        self._ax.grid(True)

        xw, yw = self._wheel_positions()
        self._wheel_x_body = xw
        self._wheel_y_body = yw

        # Robot outline (rectangle)
        self._outline_x_body = [
            0.5 * self._robot_length,
            0.5 * self._robot_length,
            -0.5 * self._robot_length,
            -0.5 * self._robot_length,
            0.5 * self._robot_length,
        ]
        self._outline_y_body = [
            0.5 * self._robot_width,
            -0.5 * self._robot_width,
            -0.5 * self._robot_width,
            0.5 * self._robot_width,
            0.5 * self._robot_width,
        ]
        (self._outline_line,) = self._ax.plot(
            self._outline_x_body, self._outline_y_body, "k-", lw=1.5
        )

        self._wheel_scatter = self._ax.scatter(
            xw, yw, c=["C1", "C1", "C1", "C1"], s=40
        )
        self._wheel_labels = []
        for i, (x, y) in enumerate(zip(xw, yw)):
            self._wheel_labels.append(
                self._ax.text(x, y, f" {i}", va="center", ha="left", fontsize=10)
            )

        u0 = [0.0, 0.0, 0.0, 0.0]
        v0 = [0.0, 0.0, 0.0, 0.0]
        self._quiver = self._ax.quiver(
            xw,
            yw,
            u0,
            v0,
            angles="xy",
            scale_units="xy",
            scale=1.0,
            color=["C0", "C0", "C0", "C0"],
            width=0.007,
        )

        # cmd_vel arrow (single vector for x/y) and omega arrow (curved)
        self._cmd_vel_quiver = self._ax.quiver(
            [0.0],
            [0.0],
            [0.0],
            [0.0],
            angles="xy",
            scale_units="xy",
            scale=1.0,
            color=["C3"],
            width=0.010,
            zorder=5,
        )
        self._omega_radius = (
            max(self._robot_length, self._robot_width) * self._omega_arc_radius_scale
        )
        self._omega_arc = Arc(
            (0.0, 0.0),
            2.0 * self._omega_radius,
            2.0 * self._omega_radius,
            theta1=0.0,
            theta2=0.0,
            color="C3",
            lw=2.0,
            alpha=0.0,
            zorder=5,
        )
        self._ax.add_patch(self._omega_arc)
        self._omega_arrow = FancyArrowPatch(
            (0.0, 0.0),
            (0.0, 0.0),
            arrowstyle="-|>",
            mutation_scale=16,
            color="C3",
            lw=2.0,
            alpha=0.0,
            zorder=6,
        )
        self._ax.add_patch(self._omega_arrow)

        if self._show_legend:
            legend_handles = [
                Line2D([0], [0], color="k", lw=1.5, label="robot outline"),
                Line2D(
                    [0],
                    [0],
                    marker="o",
                    color="w",
                    markerfacecolor="C1",
                    markersize=8,
                    label="wheel pos (0..3)",
                ),
                FancyArrowPatch(
                    (0, 0),
                    (1, 0),
                    arrowstyle="-|>",
                    mutation_scale=12,
                    color="C0",
                    lw=2.0,
                    label="swerve_drive_ref",
                ),
                FancyArrowPatch(
                    (0, 0),
                    (1, 0),
                    arrowstyle="-|>",
                    mutation_scale=12,
                    color="C3",
                    lw=2.0,
                    label="cmd_vel (vx, vy)",
                ),
                Line2D(
                    [0],
                    [0],
                    color="C3",
                    lw=2.0,
                    label="cmd_vel omega (arc)",
                ),
            ]
            self._legend = self._ax.legend(
                handles=legend_handles,
                loc="upper right",
                framealpha=0.9,
                fontsize=9,
            )

        pad = max(self._robot_length, self._robot_width) * 0.8 + 0.2
        self._ax.set_xlim(-pad, pad)
        self._ax.set_ylim(-pad, pad)

        self._status_text = self._ax_ui_status.text(
            0.02,
            0.98,
            "waiting for /swerve_drive_ref ...",
            transform=self._ax_ui_status.transAxes,
            va="top",
            ha="left",
            fontsize=9,
            bbox=dict(boxstyle="round", facecolor="white", alpha=0.8),
            family="monospace",
            wrap=True,
        )

        if self._enable_cmd_vel_publish_ui or self._enable_imu_publish_ui:
            ui_bbox = self._ax_ui_controls.get_position()

            def _ui_add_axes(rx: float, ry: float, rw: float, rh: float):
                x0 = ui_bbox.x0 + rx * ui_bbox.width
                y0 = ui_bbox.y0 + ry * ui_bbox.height
                w = rw * ui_bbox.width
                h = rh * ui_bbox.height
                return self._fig.add_axes([x0, y0, w, h])

            def _clamp(value: float, limit: float) -> float:
                lim = abs(limit)
                if lim <= 0.0:
                    return value
                return max(-lim, min(lim, value))

        if self._enable_cmd_vel_publish_ui:
            self._ax_ui_controls.text(
                0.02,
                0.98,
                "cmd_vel publish",
                transform=self._ax_ui_controls.transAxes,
                va="top",
                ha="left",
                fontsize=10,
                fontweight="bold",
            )

            ax_tb_vx = _ui_add_axes(0.06, 0.90, 0.55, 0.06)
            ax_tb_vy = _ui_add_axes(0.06, 0.83, 0.55, 0.06)
            ax_tb_om = _ui_add_axes(0.06, 0.76, 0.55, 0.06)
            ax_btn_set = _ui_add_axes(0.64, 0.76, 0.31, 0.20)

            self._tb_vx = TextBox(ax_tb_vx, "vx", initial="0.0")
            self._tb_vy = TextBox(ax_tb_vy, "vy", initial="0.0")
            self._tb_omega = TextBox(ax_tb_om, "omega", initial="0.0")
            self._btn_set = Button(ax_btn_set, "Set")

            ax_vx = _ui_add_axes(0.06, 0.68, 0.89, 0.05)
            ax_vy = _ui_add_axes(0.06, 0.61, 0.89, 0.05)
            ax_om = _ui_add_axes(0.06, 0.54, 0.89, 0.05)
            self._slider_vx = Slider(
                ax_vx,
                "vx",
                -abs(self._cmd_vel_limit_vx),
                abs(self._cmd_vel_limit_vx),
                valinit=0.0,
            )
            self._slider_vy = Slider(
                ax_vy,
                "vy",
                -abs(self._cmd_vel_limit_vy),
                abs(self._cmd_vel_limit_vy),
                valinit=0.0,
            )
            self._slider_omega = Slider(
                ax_om,
                "omega",
                -abs(self._cmd_vel_limit_omega),
                abs(self._cmd_vel_limit_omega),
                valinit=0.0,
            )

            def _on_cmd_vel_slider_change(_val) -> None:
                self._ui_vx = float(self._slider_vx.val)
                self._ui_vy = float(self._slider_vy.val)
                self._ui_omega = float(self._slider_omega.val)
                self._tb_vx.set_val(f"{self._ui_vx:.6g}")
                self._tb_vy.set_val(f"{self._ui_vy:.6g}")
                self._tb_omega.set_val(f"{self._ui_omega:.6g}")

            self._slider_vx.on_changed(_on_cmd_vel_slider_change)
            self._slider_vy.on_changed(_on_cmd_vel_slider_change)
            self._slider_omega.on_changed(_on_cmd_vel_slider_change)

            def _apply_cmd_vel_textbox_values() -> None:
                try:
                    vx = float(self._tb_vx.text)
                    vy = float(self._tb_vy.text)
                    om = float(self._tb_omega.text)
                except Exception:
                    self.get_logger().warn("Invalid number in cmd_vel text box.")
                    return

                vx = _clamp(vx, self._cmd_vel_limit_vx)
                vy = _clamp(vy, self._cmd_vel_limit_vy)
                om = _clamp(om, self._cmd_vel_limit_omega)

                self._slider_vx.set_val(vx)
                self._slider_vy.set_val(vy)
                self._slider_omega.set_val(om)

                self._tb_vx.set_val(f"{vx:.6g}")
                self._tb_vy.set_val(f"{vy:.6g}")
                self._tb_omega.set_val(f"{om:.6g}")

            self._btn_set.on_clicked(lambda _evt: _apply_cmd_vel_textbox_values())
            self._tb_vx.on_submit(lambda _text: _apply_cmd_vel_textbox_values())
            self._tb_vy.on_submit(lambda _text: _apply_cmd_vel_textbox_values())
            self._tb_omega.on_submit(lambda _text: _apply_cmd_vel_textbox_values())

            ax_btn_pub = _ui_add_axes(0.06, 0.44, 0.41, 0.08)
            ax_btn_zero = _ui_add_axes(0.54, 0.44, 0.41, 0.08)
            ax_chk_auto = _ui_add_axes(0.06, 0.34, 0.41, 0.09)
            ax_btn_sync = _ui_add_axes(0.54, 0.35, 0.41, 0.08)

            self._btn_pub = Button(ax_btn_pub, "Pub once")
            self._btn_zero = Button(ax_btn_zero, "Zero+Pub")
            self._chk_auto = CheckButtons(ax_chk_auto, ["Auto pub"], [False])
            self._btn_sync = Button(ax_btn_sync, "Sync rx")

            self._btn_pub.on_clicked(lambda _evt: self.publish_cmd_vel_once())

            def _zero_cmd_vel(_evt) -> None:
                self._slider_vx.set_val(0.0)
                self._slider_vy.set_val(0.0)
                self._slider_omega.set_val(0.0)
                self._tb_vx.set_val("0.0")
                self._tb_vy.set_val("0.0")
                self._tb_omega.set_val("0.0")
                self.publish_cmd_vel_once()

            self._btn_zero.on_clicked(_zero_cmd_vel)

            def _toggle_cmd_vel_auto(_label) -> None:
                self._cmd_vel_auto_pub_enabled = bool(self._chk_auto.get_status()[0])
                self._cmd_vel_auto_pub_last_monotonic = None

            self._chk_auto.on_clicked(_toggle_cmd_vel_auto)

            def _sync_cmd_vel(_evt) -> None:
                if self._cmd_vel is None:
                    return
                self._slider_vx.set_val(self._cmd_vel.vx)
                self._slider_vy.set_val(self._cmd_vel.vy)
                self._slider_omega.set_val(self._cmd_vel.omega)
                self._tb_vx.set_val(f"{self._cmd_vel.vx:.6g}")
                self._tb_vy.set_val(f"{self._cmd_vel.vy:.6g}")
                self._tb_omega.set_val(f"{self._cmd_vel.omega:.6g}")

            self._btn_sync.on_clicked(_sync_cmd_vel)

        if self._enable_imu_publish_ui:
            self._ax_ui_controls.text(
                0.02,
                0.30,
                "imu publish (yaw only)",
                transform=self._ax_ui_controls.transAxes,
                va="top",
                ha="left",
                fontsize=10,
                fontweight="bold",
            )

            ax_tb_yaw = _ui_add_axes(0.06, 0.22, 0.55, 0.06)
            ax_btn_yaw_set = _ui_add_axes(0.64, 0.18, 0.31, 0.12)
            self._tb_imu_yaw = TextBox(ax_tb_yaw, "yaw(rad)", initial="0.0")
            self._btn_imu_set = Button(ax_btn_yaw_set, "Set")

            ax_sl_yaw = _ui_add_axes(0.06, 0.14, 0.89, 0.05)
            self._slider_imu_yaw = Slider(
                ax_sl_yaw,
                "yaw",
                -abs(self._imu_yaw_limit),
                abs(self._imu_yaw_limit),
                valinit=0.0,
            )

            def _on_imu_slider_change(_val) -> None:
                self._ui_imu_yaw = float(self._slider_imu_yaw.val)
                self._tb_imu_yaw.set_val(f"{self._ui_imu_yaw:.6g}")

            self._slider_imu_yaw.on_changed(_on_imu_slider_change)

            def _apply_imu_textbox_values() -> None:
                try:
                    yaw = float(self._tb_imu_yaw.text)
                except Exception:
                    self.get_logger().warn("Invalid number in imu yaw text box.")
                    return
                yaw = _clamp(yaw, self._imu_yaw_limit)
                self._slider_imu_yaw.set_val(yaw)
                self._tb_imu_yaw.set_val(f"{yaw:.6g}")

            self._btn_imu_set.on_clicked(lambda _evt: _apply_imu_textbox_values())
            self._tb_imu_yaw.on_submit(lambda _text: _apply_imu_textbox_values())

            ax_imu_pub = _ui_add_axes(0.06, 0.05, 0.41, 0.07)
            ax_imu_zero = _ui_add_axes(0.54, 0.05, 0.41, 0.07)
            ax_imu_auto = _ui_add_axes(0.06, 0.00, 0.41, 0.05)
            self._btn_imu_pub = Button(ax_imu_pub, "Pub once")
            self._btn_imu_zero = Button(ax_imu_zero, "Zero+Pub")
            self._chk_imu_auto = CheckButtons(ax_imu_auto, ["Auto pub"], [False])

            self._btn_imu_pub.on_clicked(lambda _evt: self.publish_imu_once())

            def _zero_imu(_evt) -> None:
                self._slider_imu_yaw.set_val(0.0)
                self._tb_imu_yaw.set_val("0.0")
                self.publish_imu_once()

            self._btn_imu_zero.on_clicked(_zero_imu)

            def _toggle_imu_auto(_label) -> None:
                self._imu_auto_pub_enabled = bool(self._chk_imu_auto.get_status()[0])
                self._imu_auto_pub_last_monotonic = None

            self._chk_imu_auto.on_clicked(_toggle_imu_auto)

        self._closed = False

        def _on_close(_evt) -> None:
            self._closed = True

        self._fig.canvas.mpl_connect("close_event", _on_close)
        self._fig.canvas.draw()
        self._fig.canvas.flush_events()

    def _update_heading_estimate(self, now_monotonic: float) -> None:
        if not self._rotate_robot:
            self._heading_last_monotonic = now_monotonic
            return
        if self._cmd_vel is None:
            self._heading_last_monotonic = now_monotonic
            return
        if self._heading_last_monotonic is None:
            self._heading_last_monotonic = now_monotonic
            return
        dt = now_monotonic - self._heading_last_monotonic
        self._heading_last_monotonic = now_monotonic
        if dt <= 0.0:
            return
        self._heading_est += self._cmd_vel.omega * dt
        self._heading_est = (self._heading_est + math.pi) % (2.0 * math.pi) - math.pi

    def publish_cmd_vel_once(self) -> None:
        if self._cmd_vel_pub is None:
            return
        msg = Twist()
        msg.linear.x = float(self._ui_vx)
        msg.linear.y = float(self._ui_vy)
        msg.angular.z = float(self._ui_omega)
        self._cmd_vel_pub.publish(msg)

    def maybe_publish_cmd_vel(self, now_monotonic: float) -> None:
        if self._cmd_vel_pub is None:
            return
        if not self._cmd_vel_auto_pub_enabled:
            return
        if self._cmd_vel_auto_pub_last_monotonic is None:
            self._cmd_vel_auto_pub_last_monotonic = now_monotonic
            self.publish_cmd_vel_once()
            return
        period = 1.0 / self._cmd_vel_publish_rate_hz
        if now_monotonic - self._cmd_vel_auto_pub_last_monotonic >= period:
            self._cmd_vel_auto_pub_last_monotonic = now_monotonic
            self.publish_cmd_vel_once()

    def publish_imu_once(self) -> None:
        if self._imu_pub is None:
            return
        yaw = float(self._ui_imu_yaw)
        half = 0.5 * yaw
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._imu_frame_id
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = math.sin(half)
        msg.orientation.w = math.cos(half)
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = 0.0
        msg.linear_acceleration.x = 0.0
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = 0.0
        self._imu_pub.publish(msg)

    def maybe_publish_imu(self, now_monotonic: float) -> None:
        if self._imu_pub is None:
            return
        if not self._imu_auto_pub_enabled:
            return
        if self._imu_auto_pub_last_monotonic is None:
            self._imu_auto_pub_last_monotonic = now_monotonic
            self.publish_imu_once()
            return
        period = 1.0 / self._imu_publish_rate_hz
        if now_monotonic - self._imu_auto_pub_last_monotonic >= period:
            self._imu_auto_pub_last_monotonic = now_monotonic
            self.publish_imu_once()

    def update_plot(self) -> None:
        now = time.monotonic()
        self._update_heading_estimate(now)

        if self._latest is None:
            self._fig.canvas.draw_idle()
            return

        cmd = self._cmd_vel

        yaw = 0.0
        yaw_src = "none"
        if self._imu_yaw is not None:
            yaw = self._imu_yaw.yaw
            yaw_src = "imu"
        else:
            yaw = self._heading_est
            yaw_src = "cmd_vel_int"

        if self._rotate_robot:
            xw, yw = self._rotate_xy(self._wheel_x_body, self._wheel_y_body, yaw)
            ox, oy = self._rotate_xy(self._outline_x_body, self._outline_y_body, yaw)
            self._outline_line.set_data(ox, oy)
            self._wheel_scatter.set_offsets(list(zip(xw, yw)))
            for label, xi, yi in zip(self._wheel_labels, xw, yw):
                label.set_position((xi, yi))
        else:
            xw, yw = self._wheel_x_body, self._wheel_y_body

        u = []
        v = []
        for i in range(4):
            speed = self._swerve_speed_linear(self._latest.omega[i])
            theta = self._latest.theta[i] + yaw
            u.append(self._vector_scale * speed * math.cos(theta))
            v.append(self._vector_scale * speed * math.sin(theta))

        self._quiver.set_offsets(list(zip(xw, yw)))
        self._quiver.set_UVC(u, v)

        age = now - self._latest.stamp_monotonic
        lines: list[str] = []
        lines.append(f"swerve rx: {self._topic}")
        lines.append(f"cmd_vel rx: {self._cmd_vel_topic}")
        if self._enable_cmd_vel_publish_ui:
            lines.append(f"cmd_vel tx: {self._cmd_vel_pub_topic}")
        if self._enable_imu_publish_ui:
            lines.append(f"imu tx: {self._imu_pub_topic}")
        lines.append(f"robot_yaw({yaw_src}): {yaw:+.3f} rad ({math.degrees(yaw):+.1f} deg)")
        lines.append(f"swerve age: {age:.3f} s")

        if cmd is None:
            lines.append("cmd_vel: (no message)")
        else:
            cmd_age = now - cmd.stamp_monotonic
            lines.append(
                "cmd_vel: vx={:+.3f}, vy={:+.3f}, omega={:+.3f}".format(
                    cmd.vx, cmd.vy, cmd.omega
                )
            )
            lines.append(f"cmd_age: {cmd_age:.3f} s")
            move_dir = (
                math.atan2(cmd.vy, cmd.vx) if (cmd.vx != 0.0 or cmd.vy != 0.0) else 0.0
            )
            lines.append(
                "move_dir: {:+.3f} rad ({:+.1f} deg)".format(
                    move_dir, math.degrees(move_dir)
                )
            )
            lines.append(
                "turn_rate: {:+.3f} rad/s ({:+.1f} deg/s)".format(
                    cmd.omega, math.degrees(cmd.omega)
                )
            )
            lines.append(
                "turn_angle(est): {:+.3f} rad ({:+.1f} deg)".format(
                    self._heading_est, math.degrees(self._heading_est)
                )
            )

        if self._enable_cmd_vel_publish_ui:
            lines.append(
                "ui: vx={:+.3f}, vy={:+.3f}, omega={:+.3f}".format(
                    self._ui_vx, self._ui_vy, self._ui_omega
                )
            )
            lines.append(
                "auto_pub: {} @{} Hz".format(
                    "ON" if self._cmd_vel_auto_pub_enabled else "OFF",
                    int(self._cmd_vel_publish_rate_hz),
                )
            )
        if self._enable_imu_publish_ui:
            lines.append(
                "imu_yaw: {:+.3f} rad ({:+.1f} deg)".format(
                    self._ui_imu_yaw, math.degrees(self._ui_imu_yaw)
                )
            )
            lines.append(
                "imu_auto_pub: {} @{} Hz".format(
                    "ON" if self._imu_auto_pub_enabled else "OFF",
                    int(self._imu_publish_rate_hz),
                )
            )

        lines.append(
            "v(m/s): [{:+.3f}, {:+.3f}, {:+.3f}, {:+.3f}]".format(
                self._swerve_speed_linear(self._latest.omega[0]),
                self._swerve_speed_linear(self._latest.omega[1]),
                self._swerve_speed_linear(self._latest.omega[2]),
                self._swerve_speed_linear(self._latest.omega[3]),
            )
        )
        lines.append(
            "theta: [{:+.3f}, {:+.3f}, {:+.3f}, {:+.3f}] rad".format(
                self._latest.theta[0],
                self._latest.theta[1],
                self._latest.theta[2],
                self._latest.theta[3],
            )
        )
        if self._swerve_v_is_angular:
            lines.append(
                "omega_raw(rad/s): [{:+.3f}, {:+.3f}, {:+.3f}, {:+.3f}]  r={:.4g}m".format(
                    self._latest.omega[0],
                    self._latest.omega[1],
                    self._latest.omega[2],
                    self._latest.omega[3],
                    self._wheel_radius,
                )
            )

        # cmd_vel vector (x/y)
        if cmd is None:
            cvx, cvy, comega = 0.0, 0.0, 0.0
        else:
            cvx, cvy, comega = cmd.vx, cmd.vy, cmd.omega
        self._cmd_vel_quiver.set_UVC(
            [self._cmd_vel_vector_scale * cvx],
            [self._cmd_vel_vector_scale * cvy],
        )

        # omega curved arrow (about origin). Span reflects |omega|.
        omega_lim = abs(self._cmd_vel_limit_omega) if self._cmd_vel_limit_omega != 0.0 else 1.0
        omega_ratio = self._clamp01(abs(comega) / omega_lim)
        if omega_ratio < 1e-3:
            self._omega_arc.set_alpha(0.0)
            self._omega_arrow.set_alpha(0.0)
        else:
            span_deg = 45.0 + 225.0 * omega_ratio  # 45..270 deg
            base_deg = 30.0
            if comega >= 0.0:
                theta1 = base_deg
                theta2 = base_deg + span_deg
                arrow_dir = 1.0
            else:
                theta1 = base_deg - span_deg
                theta2 = base_deg
                arrow_dir = -1.0

            alpha = 0.25 + 0.75 * omega_ratio
            self._omega_arc.theta1 = theta1
            self._omega_arc.theta2 = theta2
            self._omega_arc.set_alpha(alpha)

            end_rad = math.radians(theta2)
            start_rad = math.radians(theta2 - 10.0 * arrow_dir)
            x0 = self._omega_radius * math.cos(start_rad)
            y0 = self._omega_radius * math.sin(start_rad)
            x1 = self._omega_radius * math.cos(end_rad)
            y1 = self._omega_radius * math.sin(end_rad)
            self._omega_arrow.set_positions((x0, y0), (x1, y1))
            self._omega_arrow.set_alpha(alpha)

        self._status_text.set_text("\n".join(lines))

        self._fig.canvas.draw_idle()

    @property
    def closed(self) -> bool:
        return self._closed

    @property
    def rate_hz(self) -> float:
        return self._rate_hz


def main() -> None:
    rclpy.init()
    node = SwerveDriveViewer()
    try:
        dt = 1.0 / node.rate_hz
        while rclpy.ok() and not node.closed:
            start = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0.0)
            node.maybe_publish_cmd_vel(start)
            node.maybe_publish_imu(start)
            node.update_plot()
            plt.pause(0.001)
            elapsed = time.monotonic() - start
            if elapsed < dt:
                time.sleep(dt - elapsed)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
