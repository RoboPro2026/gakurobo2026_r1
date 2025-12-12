import csv
import os
from datetime import datetime
from typing import Callable

import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
import trajectory_planner
from matplotlib.transforms import Affine2D


class RunTrajectoryPlanner:
    """軌道計画の読み込み・計算・描画をまとめたクラス"""

    def __init__(
        self,
        zone: str,
        input_prefix: str,
        output_prefix: str | None,
        dt: float,
        v_max: float,
        a_max: float,
        j_max: float,
        omega_max: float | None,
        fig: plt.Figure | None,
        ax: plt.Axes | None,
        robot_dt: float = 0.2,
        log_func: Callable[[str], None] | None = None,
    ) -> None:
        assert zone in ["red", "blue"], "zone must be 'red' or 'blue'"
        self.zone = zone
        self.input_prefix = input_prefix
        # 出力ファイルのベース名が指定されていない場合は入力と同じにする
        self.output_prefix = input_prefix if output_prefix is None else output_prefix

        self.dt = dt
        self.v_max = v_max
        self.a_max = a_max
        self.j_max = j_max
        self.omega_max = 5 * (2 * np.pi) if omega_max is None else omega_max
        self.robot_dt = robot_dt
        # GUI から渡されたログハンドラ（None の場合はコンソールのみ）
        self.log_func: Callable[[str], None] | None = log_func

        # matplotlib
        if fig is None or ax is None:
            self.fig, self.ax = plt.subplots(figsize=(12, 12))
        else:
            self.fig = fig
            self.ax = ax

        # waypoint / 軌道結果
        self.x_wp: list[float] = []
        self.y_wp: list[float] = []
        self.theta_wp: list[tuple[int, float]] = []
        self.v_trans_wp: list[tuple[int, float]] = []

        self.status = None
        self.t = None
        self.x = None
        self.y = None
        self.theta = None
        self.distance = None
        self.v_trans = None
        self.a_trans = None
        self.j_trans = None
        self.omega = None
        self.curvature = None

    def _log(self, msg: str) -> None:
        """ログ出力用ヘルパー（コンソール + GUI へ）"""
        # 常にコマンドラインへ出力
        print(msg)
        # GUI 側のログビューにも出力（設定されていれば）
        if self.log_func is not None:
            self.log_func(msg)

    def run(self, reload_waypoints: bool = True) -> bool:
        """一連の処理を実行して軌道計算まで行う

        戻り値:
            True: 軌道生成に成功（失敗ステータスなし）
            False: 軌道生成に失敗（少なくとも1区間でFAILURE）
        """
        if reload_waypoints:
            self.load_waypoint()
        self._print_waypoints()
        return self._calculate_trajectory()

    def plot(self) -> None:
        """計算済みの軌道をプロットする"""
        self._plot_field()
        self._set_axis_range()
        self._connect_click()
        self._plot_robot_along_trajectory()
        self._plot_trajectory()
        self._plot_detailed_time_series()
        plt.tight_layout()
        plt.show()

    def load_waypoint(
        self,
    ) -> tuple[
        list[float],
        list[float],
        list[tuple[int, float]],
        list[tuple[int, float]],
    ]:
        """CSV から waypoint を読み込み、リストも返す"""
        self.x_wp.clear()
        self.y_wp.clear()
        self.theta_wp.clear()
        self.v_trans_wp.clear()

        with open(self._get_input_waypoint_path(), "r") as f:
            reader = csv.reader(f)
            for i, row in enumerate(reader):
                self.x_wp.append(float(row[0]))
                self.y_wp.append(float(row[1]))
                if row[2] != "":
                    self.theta_wp.append((i, float(row[2])))
                if row[3] != "":
                    self.v_trans_wp.append((i, float(row[3])))

        return self.x_wp, self.y_wp, self.theta_wp, self.v_trans_wp

    def save_waypoint(self) -> None:
        """現在の waypoint を CSV に保存する"""
        # メインファイルの保存
        out_path = self._get_output_waypoint_path()
        self._write_waypoints_to_path(out_path)

        # バックアップファイル名を作成（例: backup_waypoints_20251212_153045.csv）
        base_dir, base_name = os.path.split(out_path)
        name, ext = os.path.splitext(base_name)
        date_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        backup_name = f"backup_{name}_{date_str}{ext}"
        backup_path = os.path.join(base_dir, backup_name)

        # バックアップとして同じ内容を書き出す
        self._write_waypoints_to_path(backup_path)

    # ========= path helpers =========
    def _get_input_waypoint_path(self) -> str:
        """入力用 waypoint ファイルパス (xxx_waypoints.csv)"""
        return f"{self.input_prefix}_waypoints.csv"

    def _get_output_waypoint_path(self) -> str:
        """出力用 waypoint ファイルパス (xxx_waypoints.csv)"""
        return f"{self.output_prefix}_waypoints.csv"

    def _get_input_parameter_path(self, parameter_path: str | None = None) -> str:
        """入力用 robot_parameter ファイルパス (xxx_robot_parameter.csv)"""
        if parameter_path is not None:
            return parameter_path
        return f"{self.input_prefix}_robot_parameter.csv"

    def _get_output_parameter_path(self, parameter_path: str | None = None) -> str:
        """出力用 robot_parameter ファイルパス (xxx_robot_parameter.csv)"""
        if parameter_path is not None:
            return parameter_path
        return f"{self.output_prefix}_robot_parameter.csv"

    def load_parameters(self, parameter_path: str | None = None) -> bool:
        """robot_parameter.csv からロボットパラメータを読み込む

        戻り値:
            True: 読み込みに成功した（ファイルが存在した）
            False: ファイルが存在しなかった
        """
        path = self._get_input_parameter_path(parameter_path)
        try:
            with open(path, "r") as f:
                reader = csv.reader(f)
                for row in reader:
                    if len(row) < 2:
                        continue
                    key, value = row[0], row[1]
                    if key == "zone":
                        if value in ["red", "blue"]:
                            self.zone = value
                    elif key == "dt":
                        self.dt = float(value)
                    elif key == "v_max":
                        self.v_max = float(value)
                    elif key == "a_max":
                        self.a_max = float(value)
                    elif key == "j_max":
                        self.j_max = float(value)
                    elif key == "omega_max":
                        self.omega_max = float(value)
                    elif key == "robot_dt":
                        self.robot_dt = float(value)
            return True
        except FileNotFoundError:
            # パラメータファイルが無い場合は何もしない
            return False

    def save_parameters(self, parameter_path: str | None = None) -> None:
        """現在のロボットパラメータを robot_parameter.csv に保存する"""
        path = self._get_output_parameter_path(parameter_path)
        # メインファイルの保存
        self._write_parameters_to_path(path)

        # バックアップファイル名を作成（例: backup_robot_parameter_20251212_153045.csv）
        base_dir, base_name = os.path.split(path)
        name, ext = os.path.splitext(base_name)
        date_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        backup_name = f"backup_{name}_{date_str}{ext}"
        backup_path = os.path.join(base_dir, backup_name)

        # バックアップとして同じ内容を書き出す
        self._write_parameters_to_path(backup_path)

    def _write_parameters_to_path(self, path: str) -> None:
        """与えられたパスに現在のパラメータを書き出す内部ヘルパー"""
        with open(path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["zone", self.zone])
            writer.writerow(["dt", self.dt])
            writer.writerow(["v_max", self.v_max])
            writer.writerow(["a_max", self.a_max])
            writer.writerow(["j_max", self.j_max])
            writer.writerow(["omega_max", self.omega_max])
            writer.writerow(["robot_dt", self.robot_dt])

    def _write_waypoints_to_path(self, path: str) -> None:
        """与えられたパスに現在の waypoint を書き出す内部ヘルパー"""
        with open(path, "w", newline="") as f:
            writer = csv.writer(f)
            j = 0
            k = 0
            for i in range(len(self.x_wp)):
                theta = ""
                if j < len(self.theta_wp) and self.theta_wp[j][0] == i:
                    theta = self.theta_wp[j][1]
                    j += 1
                v = ""
                if k < len(self.v_trans_wp) and self.v_trans_wp[k][0] == i:
                    v = self.v_trans_wp[k][1]
                    k += 1
                writer.writerow([self.x_wp[i], self.y_wp[i], theta, v])

    # ========= waypoint getters =========
    def get_waypoints(
        self,
    ) -> tuple[
        list[float],
        list[float],
        list[tuple[int, float]],
        list[tuple[int, float]],
    ]:
        """waypoint 全体をまとめて取得"""
        return self.x_wp, self.y_wp, self.theta_wp, self.v_trans_wp

    def get_x_wp(self) -> list[float]:
        return self.x_wp

    def get_y_wp(self) -> list[float]:
        return self.y_wp

    def get_theta_wp(self) -> list[tuple[int, float]]:
        return self.theta_wp

    def get_v_trans_wp(self) -> list[tuple[int, float]]:
        return self.v_trans_wp

    # ========= internal helpers =========
    def _get_zone_adjusted_waypoints(
        self,
    ) -> tuple[list[float], list[float], list[tuple[int, float]], list[tuple[int, float]]]:
        """ゾーンに応じて変換した waypoint を返す（元データは変更しない）"""
        x_adj = list(self.x_wp)
        theta_adj = list(self.theta_wp)

        if self.zone == "red":
            x_adj = [-x for x in self.x_wp]
            theta_adj = [(i, np.pi - th) for i, th in self.theta_wp]

        return x_adj, self.y_wp, theta_adj, self.v_trans_wp

    def _print_waypoints(self) -> None:
        self._log("Waypoints:")
        j = 0
        k = 0
        for i in range(len(self.x_wp)):
            s = ""
            s += f"  {i}: x={self.x_wp[i]:.3f}, y={self.y_wp[i]:.3f}"
            if j < len(self.theta_wp) and i == self.theta_wp[j][0]:
                s += f", theta={self.theta_wp[j][1]}"
                j += 1
            else:
                s += ", theta=N/A"

            if k < len(self.v_trans_wp) and i == self.v_trans_wp[k][0]:
                s += f", v_trans={self.v_trans_wp[k][1]}"
                k += 1
            else:
                s += ", v_trans=N/A"

            self._log(s)

    def _plot_object_with_zone(self, field_object: patches.Patch) -> None:
        """オブジェクトをゾーンに応じてプロット"""
        if self.zone == "red" and isinstance(field_object, patches.Rectangle):
            mirrored_x = -(field_object.get_x() + field_object.get_width())
            field_object.set_x(mirrored_x)
        self.ax.add_patch(field_object)

    def _plot_field(self) -> None:
        """フィールドの描画"""
        field = patches.Rectangle(
            xy=(0, 0),
            width=6.0,
            height=12.0,
            linewidth=1,
            edgecolor="black",
            facecolor="none",
        )
        r1_start_zone = patches.Rectangle(
            xy=(5.0, 11.0),
            width=1.0,
            height=1.0,
            linewidth=1,
            edgecolor="black",
            facecolor="none",
        )
        r2_start_zone = patches.Rectangle(
            xy=(0.875, 11.2),
            width=0.8,
            height=0.8,
            linewidth=1,
            edgecolor="black",
            facecolor="none",
        )
        retry_zone = patches.Rectangle(
            xy=(5.0, 0.0),
            width=1.0,
            height=1.0,
            linewidth=1,
            edgecolor="black",
            facecolor="none",
        )
        poll_rack = patches.Rectangle(
            xy=(3.0, 11.7),
            width=0.8,
            height=0.3,
            linewidth=1,
            edgecolor="black",
            facecolor="none",
        )
        head_rack = patches.Rectangle(
            xy=(0.0, 10.45),
            width=0.15,
            height=1.2,
            linewidth=1,
            edgecolor="black",
            facecolor="none",
        )
        hidensyo_rack = patches.Rectangle(
            xy=(0.0, 0.437),
            width=0.16,
            height=1.626,
            linewidth=1,
            edgecolor="black",
            facecolor="none",
        )
        hidensyo_rack1 = patches.Rectangle(
            xy=(0.0, 0.437),
            width=0.16,
            height=0.542,
            linewidth=1,
            edgecolor="black",
            facecolor="none",
        )
        hidensyo_rack2 = patches.Rectangle(
            xy=(0.0, 0.437),
            width=0.16,
            height=1.084,
            linewidth=1,
            edgecolor="black",
            facecolor="none",
        )
        yariokiba = patches.Rectangle(
            xy=(0.525, 2.2),
            width=1.5,
            height=0.3,
            linewidth=1,
            edgecolor="black",
            facecolor="none",
        )
        arena_entrance = patches.Rectangle(
            xy=(0.0, 2.5),
            width=4.025,
            height=0.05,
            linewidth=1,
            edgecolor="black",
            facecolor="none",
        )
        forest1 = patches.Rectangle(
            xy=(3.6, 7.6),
            width=1.2,
            height=1.2,
            linewidth=1,
            edgecolor="black",
            facecolor="none",
        )
        forest1_square = patches.Rectangle(
            xy=(3.6 + 0.428, 7.6 + 0.428),
            width=0.35,
            height=0.35,
            linewidth=1,
            edgecolor="black",
            facecolor="none",
        )
        forest2 = patches.Rectangle(
            xy=(2.4, 7.6),
            width=1.2,
            height=1.2,
            linewidth=1,
            edgecolor="black",
            facecolor="none",
        )
        forest2_square = patches.Rectangle(
            xy=(2.4 + 0.428, 7.6 + 0.428),
            width=0.35,
            height=0.35,
            linewidth=1,
            edgecolor="black",
            facecolor="none",
        )
        forest3 = patches.Rectangle(
            xy=(1.2, 7.6),
            width=1.2,
            height=1.2,
            linewidth=1,
            edgecolor="black",
            facecolor="none",
        )
        forest3_square = patches.Rectangle(
            xy=(1.2 + 0.428, 7.6 + 0.428),
            width=0.35,
            height=0.35,
            linewidth=1,
            edgecolor="black",
            facecolor="none",
        )
        forest4 = patches.Rectangle(
            xy=(3.6, 6.4),
            width=1.2,
            height=1.2,
            linewidth=1,
            edgecolor="black",
            facecolor="none",
        )
        forest4_square = patches.Rectangle(
            xy=(3.6 + 0.428, 6.4 + 0.428),
            width=0.35,
            height=0.35,
            linewidth=1,
            edgecolor="black",
            facecolor="none",
        )
        forest5 = patches.Rectangle(
            xy=(2.4, 6.4),
            width=1.2,
            height=1.2,
            linewidth=1,
            edgecolor="black",
            facecolor="none",
        )
        forest5_square = patches.Rectangle(
            xy=(2.4 + 0.428, 6.4 + 0.428),
            width=0.35,
            height=0.35,
            linewidth=1,
            edgecolor="black",
            facecolor="none",
        )
        forest6 = patches.Rectangle(
            xy=(1.2, 6.4),
            width=1.2,
            height=1.2,
            linewidth=1,
            edgecolor="black",
            facecolor="none",
        )
        forest6_square = patches.Rectangle(
            xy=(1.2 + 0.428, 6.4 + 0.428),
            width=0.35,
            height=0.35,
            linewidth=1,
            edgecolor="black",
            facecolor="none",
        )
        forest7 = patches.Rectangle(
            xy=(3.6, 5.2),
            width=1.2,
            height=1.2,
            linewidth=1,
            edgecolor="black",
            facecolor="none",
        )
        forest7_square = patches.Rectangle(
            xy=(3.6 + 0.428, 5.2 + 0.428),
            width=0.35,
            height=0.35,
            linewidth=1,
            edgecolor="black",
            facecolor="none",
        )
        forest8 = patches.Rectangle(
            xy=(2.4, 5.2),
            width=1.2,
            height=1.2,
            linewidth=1,
            edgecolor="black",
            facecolor="none",
        )
        forest8_square = patches.Rectangle(
            xy=(2.4 + 0.428, 5.2 + 0.428),
            width=0.35,
            height=0.35,
            linewidth=1,
            edgecolor="black",
            facecolor="none",
        )
        forest9 = patches.Rectangle(
            xy=(1.2, 5.2),
            width=1.2,
            height=1.2,
            linewidth=1,
            edgecolor="black",
            facecolor="none",
        )
        forest9_square = patches.Rectangle(
            xy=(1.2 + 0.428, 5.2 + 0.428),
            width=0.35,
            height=0.35,
            linewidth=1,
            edgecolor="black",
            facecolor="none",
        )
        forest10 = patches.Rectangle(
            xy=(3.6, 4.0),
            width=1.2,
            height=1.2,
            linewidth=1,
            edgecolor="black",
            facecolor="none",
        )
        forest10_square = patches.Rectangle(
            xy=(3.6 + 0.428, 4.0 + 0.428),
            width=0.35,
            height=0.35,
            linewidth=1,
            edgecolor="black",
            facecolor="none",
        )
        forest11 = patches.Rectangle(
            xy=(2.4, 4.0),
            width=1.2,
            height=1.2,
            linewidth=1,
            edgecolor="black",
            facecolor="none",
        )
        forest11_square = patches.Rectangle(
            xy=(2.4 + 0.428, 4.0 + 0.428),
            width=0.35,
            height=0.35,
            linewidth=1,
            edgecolor="black",
            facecolor="none",
        )
        forest12 = patches.Rectangle(
            xy=(1.2, 4.0),
            width=1.2,
            height=1.2,
            linewidth=1,
            edgecolor="black",
            facecolor="none",
        )
        forest12_square = patches.Rectangle(
            xy=(1.2 + 0.428, 4.0 + 0.428),
            width=0.35,
            height=0.35,
            linewidth=1,
            edgecolor="black",
            facecolor="none",
        )

        for obj in [
            field,
            r1_start_zone,
            r2_start_zone,
            retry_zone,
            poll_rack,
            head_rack,
            hidensyo_rack,
            hidensyo_rack1,
            hidensyo_rack2,
            yariokiba,
            arena_entrance,
            forest1,
            forest2,
            forest3,
            forest4,
            forest5,
            forest6,
            forest7,
            forest8,
            forest9,
            forest10,
            forest11,
            forest12,
            forest1_square,
            forest2_square,
            forest3_square,
            forest4_square,
            forest5_square,
            forest6_square,
            forest7_square,
            forest8_square,
            forest9_square,
            forest10_square,
            forest11_square,
            forest12_square,
        ]:
            self._plot_object_with_zone(obj)

    def _set_axis_range(self) -> None:
        """ゾーンに応じて表示範囲を設定"""
        if self.zone == "blue":
            self.ax.set_xlim(-1, 13)
            self.ax.set_ylim(-1, 13)
        else:
            self.ax.set_xlim(-13, 1)
            self.ax.set_ylim(-1, 13)
        # X, Y のスケールを 1:1 にする
        self.ax.set_aspect("equal", adjustable="box")

    def _connect_click(self) -> None:
        """クリックイベントを接続"""

        def on_click(event):
            if event.inaxes != self.ax:
                return
            print(
                f"次の座標がクリックされました: "
                f"({event.xdata:.3f}, {event.ydata:.3f})"
            )

        self.fig.canvas.mpl_connect("button_press_event", on_click)

    def _calculate_trajectory(self) -> bool:
        """軌道を計算し、成功/失敗を返す"""

        # ゾーンに応じた waypoint で軌道計算
        x_calc, y_calc, theta_calc, v_trans_calc = self._get_zone_adjusted_waypoints()

        (
            self.status,
            self.t,
            self.x,
            self.y,
            self.theta,
            self.distance,
            self.v_trans,
            self.a_trans,
            self.j_trans,
            self.omega,
            self.curvature,
        ) = trajectory_planner.calculate_trajectory(
            x_calc,
            y_calc,
            theta_calc,
            v_trans_calc,
            self.dt,
            self.v_max,
            self.a_max,
            self.j_max,
            self.omega_max,
        )

        # C++側の FAILURE(-3) を検出したら以降の処理を止める
        has_failure = any(st == -3 for st in self.status)

        self._log("各区間の軌道生成ステータス:")
        for i, st in enumerate(self.status):
            if st == 0:
                self._log(f"Segment {i}: 軌道生成に成功しました")
            elif st == -1:
                self._log(f"Segment {i}: 警告、目標速度に達していません")
            elif st == -2:
                self._log(f"Segment {i}: 警告、最大角速度を超えています")
            elif st == -3:
                self._log(f"Segment {i}: 失敗、軌道生成に失敗しました")

        if has_failure:
            self._log("軌道生成に失敗した区間があるため、以降の処理を中止します。")
            # 失敗時は距離・時間などを参照しないように False を返す
            return False

        # 正常に生成できている場合のみ、距離・時間を計算
        if self.distance and self.t:
            distance_total = self.distance[-1]
            time_total = self.t[-1]
            self._log(
                f"軌道全体の距離: {distance_total:.3f} [m], "
                f"軌道全体の時間: {time_total:.3f} [s]"
            )

        return True

    def _plot_robot(
        self, x: float, y: float, theta: float, width: float, height: float
    ) -> None:
        """ロボット矩形を 1 つ描画"""
        rect = patches.Rectangle(
            (-width / 2, -height / 2),
            width,
            height,
            facecolor="none",
            edgecolor="blue",
            linewidth=2,
        )

        trans = Affine2D().rotate(theta).translate(x, y) + self.ax.transData
        rect.set_transform(trans)
        self.ax.add_patch(rect)

        nose = patches.Polygon(
            [
                (width / 2, 0.0),
                (width / 2 - width * 0.3, height / 4),
                (width / 2 - width * 0.3, -height / 4),
            ],
            closed=True,
            facecolor="red",
            edgecolor="red",
        )
        nose.set_transform(trans)
        self.ax.add_patch(nose)

    def _plot_robot_along_trajectory(self) -> None:
        """軌道上のロボット姿勢を間引き描画"""
        robot_width = 0.8
        robot_height = 0.8

        robot_x: list[float] = []
        robot_y: list[float] = []
        robot_theta: list[float] = []

        for i in range(0, len(self.t), int(self.robot_dt / self.dt)):
            robot_x.append(self.x[i])
            robot_y.append(self.y[i])
            robot_theta.append(self.theta[i])

        robot_x.append(self.x[-1])
        robot_y.append(self.y[-1])
        robot_theta.append(self.theta[-1])

        for x_val, y_val, th in zip(robot_x, robot_y, robot_theta):
            self._plot_robot(x_val, y_val, th, robot_width, robot_height)

    def _plot_trajectory(self) -> None:
        """軌道を 2D 平面上にプロット"""
        scatter = self.ax.scatter(
            self.x, self.y, c=self.v_trans, cmap="viridis", s=15, label="Trajectory"
        )
        # ゾーンに応じて変換した waypoint を描画
        x_wp_plot, y_wp_plot, _, _ = self._get_zone_adjusted_waypoints()
        self.ax.scatter(
            x_wp_plot,
            y_wp_plot,
            color="red",
            marker="x",
            s=40,
            label="Waypoints",
            zorder=5,
        )
        self.fig.colorbar(scatter, ax=self.ax, label="Translational Velocity (v_trans)")
        self.ax.set_xlabel("Position (x)")
        self.ax.set_ylabel("Position (y)")
        self.ax.legend()
        self.ax.grid(True, linestyle="--", alpha=0.3)

    def _plot_detailed_time_series(self) -> None:
        """位置や速度などの時系列グラフを別 Figure に描画"""
        fig, axes = plt.subplots(figsize=(10, 12), nrows=6, ncols=1, sharex=True)

        axes[0].plot(self.t, self.x, label="Position (x)")
        axes[0].set_ylabel("x [m]")

        axes[1].plot(self.t, self.y, label="Position (y)", color="orange")
        axes[1].set_ylabel("y [m]")

        axes[2].plot(self.t, self.theta, label="Orientation (theta)", color="green")
        axes[2].set_ylabel("theta [rad]")

        axes[3].plot(self.t, self.distance, label="Distance", color="purple")
        axes[3].set_ylabel("dist [m]")

        axes[4].plot(
            self.t,
            self.v_trans,
            label="Translational Velocity (v_trans)",
            color="brown",
        )
        axes[4].set_ylabel("v [m/s]")

        axes[5].plot(self.t, self.omega, label="Angular Velocity (omega)", color="pink")
        axes[5].set_ylabel("omega [rad/s]")
        axes[5].set_xlabel("Time (s)")

        for ax in axes[:-1]:
            ax.label_outer()

        fig.tight_layout(h_pad=0.2)
        plt.show()

    def plot_vel_acc_jerk(self) -> None:
        """速度・加速度・躍度をまとめて描画"""
        if self.t is None or self.v_trans is None or self.a_trans is None or self.j_trans is None:
            return

        fig, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 8))

        axes[0].plot(self.t, self.v_trans, color="brown")
        axes[0].set_ylabel("v [m/s]")

        axes[1].plot(self.t, self.a_trans, color="red")
        axes[1].set_ylabel("a [m/s^2]")

        axes[2].plot(self.t, self.j_trans, color="gray")
        axes[2].set_ylabel("j [m/s^3]")
        axes[2].set_xlabel("Time (s)")

        for ax in axes[:-1]:
            ax.label_outer()

        fig.tight_layout()
        plt.show()

    def plot_curvature(self) -> None:
        """曲率を描画"""
        if self.t is None or self.curvature is None:
            return

        plt.figure(figsize=(10, 4))
        plt.plot(self.t, self.curvature, color="black")
        plt.ylabel("curv [1/m]")
        plt.xlabel("Time (s)")
        plt.grid(True, linestyle="--", alpha=0.3)
        plt.tight_layout()
        plt.show()


def get_fig(
    zone: str = "blue",
    base_prefix: str = "/tmp/trajectory",
) -> plt.Figure:
    """GUI から再利用しやすいように Figure を返すヘルパー"""
    runner = RunTrajectoryPlanner(
        zone=zone,
        input_prefix=base_prefix,
        output_prefix=base_prefix,
        dt=0.01,
        v_max=5.0,
        a_max=5.0,
        j_max=10.0,
        omega_max=None,
        fig=None,
        ax=None,
        log_func=None,
    )
    runner.run()
    runner._plot_field()
    runner._set_axis_range()
    runner._plot_robot_along_trajectory()
    runner._plot_trajectory()
    runner.fig.tight_layout()
    return runner.fig


if __name__ == "__main__":
    runner = RunTrajectoryPlanner(
        zone="blue",
        input_prefix="/tmp/trajectory",
        output_prefix="/tmp/trajectory",
        dt=0.01,
        v_max=5.0,
        a_max=5.0,
        j_max=10.0,
        omega_max=None,
        fig=None,
        ax=None,
        log_func=None,
    )
    if runner.run():
        runner.plot()
