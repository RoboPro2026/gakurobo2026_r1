import math
import os
import sys

import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from matplotlib.transforms import Affine2D
from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (
    QApplication,
    QButtonGroup,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPushButton,
    QSlider,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)
from run_trajectory_planner import RunTrajectoryPlanner


class MainWindow(QMainWindow):
    WAYPOINT_DISPLAY_SIG_DIGITS = 15

    def __init__(self) -> None:
        super().__init__()

        # 中央ウィジェット
        widget = QWidget()
        layout = QVBoxLayout(widget)

        # ボタン行（上段: 設定関連）
        button_layout = QHBoxLayout()
        self.button_load = QPushButton("設定読み込み")
        self.button_generate = QPushButton("軌道生成")
        self.button_save = QPushButton("設定書き込み")

        self.button_load.clicked.connect(self.on_load_clicked)
        self.button_generate.clicked.connect(self.on_generate_clicked)
        self.button_save.clicked.connect(self.on_save_clicked)

        button_layout.addWidget(self.button_load)
        button_layout.addWidget(self.button_generate)
        button_layout.addWidget(self.button_save)

        # 距離・時間表示と詳細グラフボタン
        info_layout = QHBoxLayout()
        self.label_distance = QLabel("距離: N/A")
        self.label_time = QLabel("時間: N/A")
        self.button_vel_acc_jerk = QPushButton("速度/加速度/躍度")
        self.button_vel_acc_jerk.clicked.connect(self.on_vel_acc_jerk_clicked)
        self.button_curvature = QPushButton("曲率")
        self.button_curvature.clicked.connect(self.on_curvature_clicked)
        self.button_detail = QPushButton("概要グラフ")
        self.button_detail.clicked.connect(self.on_detail_clicked)
        self.button_anim = QPushButton("アニメ再生")
        self.button_anim.clicked.connect(self.on_anim_clicked)

        info_layout.addWidget(self.label_distance)
        info_layout.addWidget(self.label_time)
        info_layout.addStretch()
        info_layout.addWidget(self.button_vel_acc_jerk)
        info_layout.addWidget(self.button_curvature)
        info_layout.addWidget(self.button_detail)
        info_layout.addWidget(self.button_anim)

        # アニメーション中の情報表示（matplotlib のすぐ上に配置）
        self.label_anim_info = QLabel("t: N/A  x: N/A  y: N/A  theta: N/A  v: N/A")
        anim_info_layout = QHBoxLayout()
        anim_info_layout.addWidget(self.label_anim_info)
        anim_info_layout.addStretch()

        # ズーム・移動用スクロールバー（スライダー）
        zoom_layout = QHBoxLayout()
        zoom_layout.addWidget(QLabel("ズーム"))
        self.zoom_slider = QSlider(Qt.Orientation.Horizontal)
        self.zoom_slider.setRange(0, 100)
        self.zoom_slider.setValue(0)
        self.zoom_slider.valueChanged.connect(self.on_zoom_slider_changed)
        zoom_layout.addWidget(self.zoom_slider)

        pan_layout = QHBoxLayout()
        pan_layout.addWidget(QLabel("X移動"))
        self.pan_x_slider = QSlider(Qt.Orientation.Horizontal)
        self.pan_x_slider.setRange(-100, 100)
        self.pan_x_slider.setValue(0)
        self.pan_x_slider.valueChanged.connect(self.on_pan_slider_changed)
        pan_layout.addWidget(self.pan_x_slider)

        pan_layout.addWidget(QLabel("Y移動"))
        self.pan_y_slider = QSlider(Qt.Orientation.Horizontal)
        self.pan_y_slider.setRange(-100, 100)
        self.pan_y_slider.setValue(0)
        self.pan_y_slider.valueChanged.connect(self.on_pan_slider_changed)
        pan_layout.addWidget(self.pan_y_slider)

        # ボタン行（下段: waypoint 編集）
        waypoint_button_layout = QHBoxLayout()
        self.button_edit = QPushButton("編集")
        self.button_insert_above = QPushButton("一つ上に挿入")
        self.button_insert_below = QPushButton("一つ下に挿入")
        self.button_undo = QPushButton("戻る")
        self.button_redo = QPushButton("進む")
        self.button_delete = QPushButton("選択を削除")
        self.button_up = QPushButton("一つ上へ")
        self.button_down = QPushButton("一つ下へ")

        # クリック操作モード（排他的に切り替え）
        for b in [self.button_edit, self.button_insert_above, self.button_insert_below]:
            b.setCheckable(True)
        self._waypoint_mode_group = QButtonGroup(self)
        self._waypoint_mode_group.setExclusive(True)
        self._waypoint_mode_group.addButton(self.button_edit)
        self._waypoint_mode_group.addButton(self.button_insert_above)
        self._waypoint_mode_group.addButton(self.button_insert_below)
        self.button_edit.setChecked(True)  # 初期値は「編集」
        self._waypoint_mode_group.buttonToggled.connect(self.on_waypoint_mode_toggled)

        self.button_undo.clicked.connect(self.on_undo_clicked)
        self.button_redo.clicked.connect(self.on_redo_clicked)
        self.button_delete.clicked.connect(self.on_delete_clicked)
        self.button_up.clicked.connect(self.on_up_clicked)
        self.button_down.clicked.connect(self.on_down_clicked)

        waypoint_button_layout.addWidget(self.button_edit)
        waypoint_button_layout.addWidget(self.button_insert_above)
        waypoint_button_layout.addWidget(self.button_insert_below)
        waypoint_button_layout.addWidget(self.button_undo)
        waypoint_button_layout.addWidget(self.button_redo)
        waypoint_button_layout.addWidget(self.button_delete)
        waypoint_button_layout.addWidget(self.button_up)
        waypoint_button_layout.addWidget(self.button_down)

        # matplotlib Figure / Canvas 生成（少し大きめ、正方形寄り）
        self.fig = Figure(figsize=(6, 6), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.canvas = FigureCanvasQTAgg(self.fig)

        # 設定表示用テーブル
        self.table = QTableWidget()
        self.table.setColumnCount(5)
        self.table.setHorizontalHeaderLabels(
            ["Index", "x [m]", "y [m]", "theta [rad]", "v_trans [m/s]"]
        )
        self.table.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.Stretch
        )
        # Index 列は内部管理用なので表示しない
        self.table.setColumnHidden(0, True)
        # 行選択時にグラフ上の waypoint をハイライト
        self.table.itemSelectionChanged.connect(self.on_waypoint_selection_changed)
        # waypoint のセル編集をプロットへ即時反映
        self.table.itemChanged.connect(self.on_waypoint_item_changed)

        # パラメータ表示用テーブル
        self.param_table = QTableWidget()
        self.param_table.setColumnCount(2)
        self.param_table.setHorizontalHeaderLabels(["Parameter", "Value"])
        self.param_table.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.ResizeToContents
        )
        # 横幅を取りすぎないように制限
        self.param_table.setMaximumWidth(350)
        # 表示範囲を少し縦に広げる
        self.param_table.setMinimumHeight(260)
        # パラメータ編集（特に zone）を即時反映する
        self.param_table.itemChanged.connect(self.on_param_item_changed)

        # ファイル名ベース入力（入出力）
        self.input_prefix_edit = QLineEdit()
        self.output_prefix_edit = QLineEdit()
        default_prefix = "src/gakurobo2026_r1/data/blue/0"
        self.input_prefix_edit.setText(default_prefix)
        self.output_prefix_edit.setText(default_prefix)
        self.input_prefix_edit.setPlaceholderText(
            "入力ファイルベース（例: src/gakurobo2026_r1/data/blue/0）"
        )
        self.output_prefix_edit.setPlaceholderText(
            "出力ファイルベース（例: src/gakurobo2026_r1/data/blue/0）"
        )

        # ログ表示エリア
        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setPlaceholderText("ログ出力...")
        self.log_view.setMinimumHeight(180)

        # RunTrajectoryPlanner インスタンス
        self.runner = RunTrajectoryPlanner(
            zone="blue",
            input_prefix=self.input_prefix_edit.text(),
            output_prefix=self.output_prefix_edit.text(),
            dt=0.01,
            v_max=5.0,
            a_max=5.0,
            j_max=10.0,
            omega_max=None,
            fig=self.fig,
            ax=self.ax,
            log_func=self.append_log,
        )
        # 初期状態ではパラメータは未設定として非表示にしておく
        self.param_table.hide()

        # グラフクリックで waypoint 追加
        self.canvas.mpl_connect("button_press_event", self.on_canvas_clicked)

        # --- 右ペイン（パラメータ/waypoint） ---
        param_panel = QWidget()
        param_panel_layout = QVBoxLayout(param_panel)
        param_panel_layout.addWidget(QLabel("入力ファイルベース"))
        param_panel_layout.addWidget(self.input_prefix_edit)
        param_panel_layout.addWidget(QLabel("出力ファイルベース"))
        param_panel_layout.addWidget(self.output_prefix_edit)
        param_panel_layout.addWidget(self.param_table, 1)

        waypoint_panel = QWidget()
        waypoint_panel_layout = QVBoxLayout(waypoint_panel)
        waypoint_panel_layout.addLayout(waypoint_button_layout)
        waypoint_panel_layout.addWidget(self.table, 1)

        self.right_splitter = QSplitter(Qt.Orientation.Vertical)
        self.right_splitter.addWidget(param_panel)
        self.right_splitter.addWidget(waypoint_panel)
        self.right_splitter.setSizes([260, 500])

        # --- 上部（グラフ + 右に waypoint/parameter） ---
        self.plot_splitter = QSplitter(Qt.Orientation.Horizontal)
        self.plot_splitter.addWidget(self.canvas)
        self.plot_splitter.addWidget(self.right_splitter)
        self.plot_splitter.setSizes([900, 450])

        # --- 全体（上: plot_splitter / 下: log） ---
        self.main_splitter = QSplitter(Qt.Orientation.Vertical)
        self.main_splitter.addWidget(self.plot_splitter)
        self.main_splitter.addWidget(self.log_view)
        self.main_splitter.setSizes([800, 220])

        # レイアウト順:
        #  1. 設定ボタン
        #  2. 距離・時間表示 + グラフボタン
        #  3. ズーム・移動スライダー
        #  4. グラフ（左） + waypoint/parameter（右）
        #  5. ログ（下）
        layout.addLayout(button_layout)
        layout.addLayout(info_layout)
        layout.addLayout(zoom_layout)
        layout.addLayout(pan_layout)
        layout.addLayout(anim_info_layout)
        layout.addWidget(self.main_splitter)
        self.setCentralWidget(widget)

        self.setWindowTitle("Trajectory Planner Viewer")
        self.resize(900, 800)

        # ズーム基準範囲
        self.base_xlim = None
        self.base_ylim = None

        # アニメ/プレビュー用
        self.anim = None
        self.anim_rect = None
        self.anim_nose = None
        self.anim_frames = None
        self.robot_preview_patches = []
        self._anim_prev_label_text = None

        # 初期表示としてフィールドのみ描画（アスペクト比 1:1）
        self.draw_waypoints_only()

        # waypoint undo/redo
        self._waypoint_undo_stack: list[dict] = []
        self._waypoint_redo_stack: list[dict] = []

        self._waypoint_click_mode = "edit"

    def reset_view(self) -> None:
        """ズーム・パンの状態をリセットし、次回描画で表示範囲も再計算させる"""
        self.base_xlim = None
        self.base_ylim = None
        # スライダーも初期化
        self.zoom_slider.blockSignals(True)
        self.pan_x_slider.blockSignals(True)
        self.pan_y_slider.blockSignals(True)
        try:
            self.zoom_slider.setValue(0)
            self.pan_x_slider.setValue(0)
            self.pan_y_slider.setValue(0)
        finally:
            self.zoom_slider.blockSignals(False)
            self.pan_x_slider.blockSignals(False)
            self.pan_y_slider.blockSignals(False)

    def log_action(self, msg: str) -> None:
        """GUI操作のログを GUI とコマンドラインに出す"""
        # RunTrajectoryPlanner._log は「コマンドライン + GUI(log_func)」に出力する
        self.runner._log(f"[GUI] {msg}")

    # ===== ボタンクリックハンドラ =====
    def on_load_clicked(self) -> None:
        """設定読み込みボタン: パラメータと waypoint を読み込む"""
        self.log_action("設定読み込み: 開始")
        # ファイル名ベースを runner に反映
        self.update_runner_prefixes_from_edits()
        self.log_action(
            f"入力={self.runner.input_prefix} / 出力={self.runner.output_prefix}"
        )

        # パラメータ読み込み
        self.log_action(f"パラメータ読込: {self.runner._get_input_parameter_path()}")
        prev_zone = self.runner.zone
        if self.runner.load_parameters():
            self.update_param_table()
            self.param_table.show()
            self.log_action("パラメータ読込: OK")
        else:
            # パラメータファイルが無い場合はテーブルを隠したままにする
            self.param_table.hide()
            self.log_action("パラメータ読込: ファイルなし")

        # zone が変わった場合は、表示範囲（xlim）を保持したまま再描画すると
        # フィールドが画面外に出てしまうため、ズーム・パンをリセットして描画し直す
        if self.runner.zone != prev_zone:
            self.log_action(
                f"zone変更: {prev_zone} -> {self.runner.zone} (表示リセット)"
            )
            self.reset_view()

        # waypoint 読み込み
        self.log_action(f"waypoint読込: {self.runner._get_input_waypoint_path()}")
        self.runner.load_waypoint()
        self.update_waypoint_table()
        self.draw_waypoints_only()
        self._waypoint_undo_stack.clear()
        self._waypoint_redo_stack.clear()
        self.log_action(f"waypoint読込: {len(self.runner.x_wp)} 点")
        self.log_action("設定読み込み: 完了")

    def on_generate_clicked(self) -> None:
        """軌道生成ボタン: 軌道計算と描画を行う"""
        self.log_action("軌道生成: 開始")
        self.stop_animation(log_message=None)
        # ファイル名ベースを runner に反映
        self.update_runner_prefixes_from_edits()
        # テーブル内容を runner に反映してから軌道計算
        self.update_runner_from_table()
        # パラメータテーブルの内容も反映
        self.update_runner_params_from_table()
        self.log_action(
            f"zone={self.runner.zone}, dt={self.runner.dt}, v_max={self.runner.v_max}, "
            f"a_max={self.runner.a_max}, j_max={self.runner.j_max}, omega_max={self.runner.omega_max}"
        )
        ok = self.runner.run(reload_waypoints=False)

        # 失敗した場合はグラフ描画・距離/時間表示を行わない
        if not ok:
            self.label_distance.setText("距離: N/A")
            self.label_time.setText("時間: N/A")
            self.log_action("軌道生成: 失敗")
            return

        # Figure 全体をクリアしてから描画を更新
        self.fig.clear()
        self.ax = self.fig.add_subplot(111)
        self.runner.fig = self.fig
        self.runner.ax = self.ax
        self.runner._plot_field()
        self.runner._set_axis_range()
        # ズーム基準範囲を更新
        self.base_xlim = self.ax.get_xlim()
        self.base_ylim = self.ax.get_ylim()
        # 「未来のロボット青枠」プレビューを管理できるよう、追加された patch を記録する
        before_patches = list(self.ax.patches)
        self.runner._plot_robot_along_trajectory()
        after_patches = list(self.ax.patches)
        self.robot_preview_patches = [
            p for p in after_patches if p not in before_patches
        ]
        self.runner._plot_trajectory()
        self.canvas.draw()

        # 距離と時間を表示
        if self.runner.distance is not None and self.runner.t is not None:
            distance_total = self.runner.distance[-1]
            time_total = self.runner.t[-1]
            self.label_distance.setText(f"距離: {distance_total:.3f} [m]")
            self.label_time.setText(f"時間: {time_total:.3f} [s]")
            self.log_action(
                f"軌道生成: 完了 (距離={distance_total:.3f} m, 時間={time_total:.3f} s)"
            )

    def on_save_clicked(self) -> None:
        """設定書き込みボタン: パラメータと waypoint を保存する"""
        self.log_action("設定書き込み: 開始")
        # ファイル名ベースを runner に反映
        self.update_runner_prefixes_from_edits()
        self.log_action(
            f"入力={self.runner.input_prefix} / 出力={self.runner.output_prefix}"
        )
        # テーブル内容を runner に反映してから保存
        self.update_runner_from_table()
        self.update_runner_params_from_table()

        # 保存
        self.log_action(f"パラメータ保存: {self.runner._get_output_parameter_path()}")
        self.runner.save_parameters()
        self.log_action(f"waypoint保存: {self.runner._get_output_waypoint_path()}")
        self.runner.save_waypoint()

        # 保存後は現在の runner の値でパラメータを表示する
        self.update_param_table()
        self.param_table.show()
        self.log_action("設定書き込み: 完了")

    def append_log(self, msg: str) -> None:
        """RunTrajectoryPlanner からのログを GUI に表示"""
        self.log_view.append(msg)

    def on_vel_acc_jerk_clicked(self) -> None:
        """速度・加速度・躍度グラフ表示ボタン"""
        if self.runner.t is None or self.runner.v_trans is None:
            self.log_action("速度/加速度/躍度: 軌道未生成のため表示できません")
            return
        self.log_action("速度/加速度/躍度: 表示")
        self.runner.plot_vel_acc_jerk()

    def on_curvature_clicked(self) -> None:
        """曲率グラフ表示ボタン"""
        if self.runner.t is None or self.runner.curvature is None:
            self.log_action("曲率: 軌道未生成のため表示できません")
            return
        self.log_action("曲率: 表示")
        self.runner.plot_curvature()

    def on_detail_clicked(self) -> None:
        """詳細グラフ表示ボタン: 速度・加速度などのグラフを表示"""
        # 軌道がまだ生成されていない場合は何もしない
        if self.runner.t is None or self.runner.v_trans is None:
            self.log_action("概要グラフ: 軌道未生成のため表示できません")
            return
        self.log_action("概要グラフ: 表示")
        self.runner._plot_detailed_time_series()

    def on_anim_clicked(self) -> None:
        """アニメ再生/停止ボタン"""
        if getattr(self, "anim", None) is not None:
            self.log_action("アニメ: 停止")
            self.stop_animation(log_message=None)
            return
        self.log_action("アニメ: 再生")
        self.start_animation()

    def start_animation(self) -> None:
        """ロボット姿勢を matplotlib 上でアニメーション再生する"""
        # 軌道がまだ生成されていない場合は何もしない
        if (
            self.runner.t is None
            or self.runner.x is None
            or self.runner.y is None
            or self.runner.theta is None
        ):
            self.log_action("アニメ: 軌道未生成のため再生できません")
            return

        # 既存アニメが残っていたら停止
        self.stop_animation(log_message=None)

        # 再生開始前の表示を保存（終了後に戻す）
        self._anim_prev_label_text = self.label_anim_info.text()

        # 未来のロボット枠（プレビュー）は、再生中は非表示にする
        for p in self.robot_preview_patches:
            try:
                p.set_visible(False)
            except Exception:
                pass

        # 描画パッチを用意（1つだけ更新し続ける）
        if not hasattr(self, "anim_rect") or self.anim_rect is None:
            robot_width = 0.8
            robot_height = 0.8

            self.anim_rect = patches.Rectangle(
                (-robot_width / 2, -robot_height / 2),
                robot_width,
                robot_height,
                facecolor="none",
                edgecolor="blue",
                linewidth=2,
            )
            self.ax.add_patch(self.anim_rect)

            self.anim_nose = patches.Polygon(
                [
                    (robot_width / 2, 0.0),
                    (robot_width / 2 - robot_width * 0.3, robot_height / 4),
                    (robot_width / 2 - robot_width * 0.3, -robot_height / 4),
                ],
                closed=True,
                facecolor="red",
                edgecolor="red",
            )
            self.ax.add_patch(self.anim_nose)
        else:
            self.anim_rect.set_visible(True)
            self.anim_nose.set_visible(True)

        # フレームインデックス
        # - 再生時間を実時間と一致させるため、interval は「(step * dt)」にする
        # - 負荷を下げるため、上限 FPS を 30 程度に制限する（必要なら step が増える）
        dt = float(self.runner.dt)
        if dt <= 0.0:
            self.log_action("アニメ: dt が不正のため再生できません")
            return
        max_fps = 30.0
        min_interval = 1.0 / max_fps
        step = max(1, int(math.ceil(min_interval / dt)))
        frames = list(range(0, len(self.runner.t), step))
        if frames and frames[-1] != len(self.runner.t) - 1:
            frames.append(len(self.runner.t) - 1)
        self.anim_frames = frames

        interval_ms = max(1, int(round(step * dt * 1000.0)))
        self.anim = FuncAnimation(
            self.fig,
            self._update_animation_frame,
            frames=frames,
            interval=interval_ms,
            repeat=False,
            blit=False,
        )

        # ボタン表示更新
        self.button_anim.setText("アニメ停止")
        self.canvas.draw()
        self.log_action(
            f"アニメ: 開始 (dt={dt}, step={step}, interval_ms={interval_ms}, frames={len(frames)})"
        )

    def _update_animation_frame(self, idx: int):
        x = self.runner.x[idx]
        y = self.runner.y[idx]
        theta = self.runner.theta[idx]
        theta_disp = self.runner.get_display_theta(theta)
        v = None
        if self.runner.v_trans is not None and idx < len(self.runner.v_trans):
            v = self.runner.v_trans[idx]

        t = None
        if self.runner.t is not None and idx < len(self.runner.t):
            t = self.runner.t[idx]

        if t is None:
            self.label_anim_info.setText(
                f"t: N/A  x: {x:.3f}  y: {y:.3f}  theta: {theta_disp:.3f}  v: "
                f"{'N/A' if v is None else f'{v:.3f}'}"
            )
        else:
            self.label_anim_info.setText(
                f"t: {t:.3f}  x: {x:.3f}  y: {y:.3f}  theta: {theta_disp:.3f}  v: "
                f"{'N/A' if v is None else f'{v:.3f}'}"
            )

        trans = Affine2D().rotate(theta_disp).translate(x, y) + self.ax.transData
        self.anim_rect.set_transform(trans)
        self.anim_nose.set_transform(trans)

        # 最終フレームで元の表示に戻す
        if self.anim_frames is not None and idx == self.anim_frames[-1]:
            self.stop_animation(log_message="アニメ: 完了")
        return self.anim_rect, self.anim_nose

    def stop_animation(self, log_message: str | None = None) -> None:
        """再生中のアニメーションを停止する"""
        anim = getattr(self, "anim", None)
        if anim is not None:
            try:
                anim.event_source.stop()
            except Exception:
                pass
        self.anim = None
        self.anim_frames = None

        if hasattr(self, "anim_rect") and self.anim_rect is not None:
            self.anim_rect.set_visible(False)
        if hasattr(self, "anim_nose") and self.anim_nose is not None:
            self.anim_nose.set_visible(False)
        # Axes の作り直しに備えて参照もクリア（再生時に作り直す）
        self.anim_rect = None
        self.anim_nose = None

        # 未来のロボット枠（プレビュー）を再表示
        for p in getattr(self, "robot_preview_patches", []):
            try:
                p.set_visible(True)
            except Exception:
                pass

        # 再生前の表示に戻す
        if self._anim_prev_label_text is not None:
            self.label_anim_info.setText(self._anim_prev_label_text)
            self._anim_prev_label_text = None

        if hasattr(self, "button_anim"):
            self.button_anim.setText("アニメ再生")
        self.canvas.draw()
        if log_message is not None:
            self.log_action(log_message)

    def on_zoom_slider_changed(self, value: int) -> None:
        """ズームスライダー操作時に表示範囲を変更する"""
        self._update_view_range()

    def on_pan_slider_changed(self, value: int) -> None:
        """パン（移動）スライダー操作時に表示範囲を変更する"""
        self._update_view_range()

    def _update_view_range(self) -> None:
        """ズーム・パン設定から表示範囲を更新する"""
        if self.base_xlim is None or self.base_ylim is None:
            return

        zoom_val = self.zoom_slider.value()
        pan_x_val = self.pan_x_slider.value()
        pan_y_val = self.pan_y_slider.value()

        # zoom_val: 0..100 -> zoom factor: 1x .. 6x
        zoom = 1.0 + (zoom_val / 20.0)
        scale = 1.0 / zoom

        base_xmin, base_xmax = self.base_xlim
        base_ymin, base_ymax = self.base_ylim
        base_width = base_xmax - base_xmin
        base_height = base_ymax - base_ymin

        visible_width = base_width * scale
        visible_height = base_height * scale

        base_x_center = (base_xmin + base_xmax) / 2.0
        base_y_center = (base_ymin + base_ymax) / 2.0

        # パンの許容量（ウィンドウがベース範囲の外に出ないように制限）
        max_shift_x = max((base_width - visible_width) / 2.0, 0.0)
        max_shift_y = max((base_height - visible_height) / 2.0, 0.0)

        shift_x = (pan_x_val / 100.0) * max_shift_x
        shift_y = (pan_y_val / 100.0) * max_shift_y

        x_center = base_x_center + shift_x
        y_center = base_y_center + shift_y

        x_half = visible_width / 2.0
        y_half = visible_height / 2.0

        self.ax.set_xlim(x_center - x_half, x_center + x_half)
        self.ax.set_ylim(y_center - y_half, y_center + y_half)
        self.ax.set_aspect("equal", adjustable="box")
        self.canvas.draw()

    def on_delete_clicked(self) -> None:
        """選択された waypoint を削除する"""
        row = self.table.currentRow()
        if row < 0 or row >= len(self.runner.x_wp):
            self.log_action("waypoint削除: 行が選択されていません")
            return
        self._push_waypoint_undo()
        self.log_action(f"waypoint削除: row={row}")

        # x, y の削除
        del self.runner.x_wp[row]
        del self.runner.y_wp[row]

        # theta, v_trans も同じ index で削除
        del self.runner.theta_wp[row]
        del self.runner.v_trans_wp[row]

        self.update_waypoint_table()
        self.draw_waypoints_only()

    def on_waypoint_mode_toggled(self, button: QPushButton, checked: bool) -> None:
        """クリック操作モード（編集/挿入）を切り替える"""
        if not checked:
            return
        if button is self.button_insert_above:
            self._waypoint_click_mode = "insert_above"
            self.log_action("モード切替: 一つ上に挿入")
        elif button is self.button_insert_below:
            self._waypoint_click_mode = "insert_below"
            self.log_action("モード切替: 一つ下に挿入")
        else:
            self._waypoint_click_mode = "edit"
            self.log_action("モード切替: 編集")

    def _insert_waypoint_at(self, insert_index: int, x_val: float, y_val: float) -> None:
        """指定 index に waypoint を挿入し、theta/v_trans の index を補正する"""
        insert_index = max(0, min(insert_index, len(self.runner.x_wp)))
        self.runner.x_wp.insert(insert_index, x_val)
        self.runner.y_wp.insert(insert_index, y_val)

        self.runner.theta_wp.insert(insert_index, math.inf)
        self.runner.v_trans_wp.insert(insert_index, math.inf)

        self.update_waypoint_table()
        self.table.selectRow(insert_index)
        self.draw_waypoints_only()

    def on_up_clicked(self) -> None:
        """選択された waypoint を一つ上に移動する"""
        row = self.table.currentRow()
        if row <= 0 or row >= len(self.runner.x_wp):
            self.log_action("waypoint上へ: 移動できません")
            return
        self._push_waypoint_undo()
        self.log_action(f"waypoint上へ: row={row} -> {row-1}")

        # x, y の入れ替え
        self.runner.x_wp[row - 1], self.runner.x_wp[row] = (
            self.runner.x_wp[row],
            self.runner.x_wp[row - 1],
        )
        self.runner.y_wp[row - 1], self.runner.y_wp[row] = (
            self.runner.y_wp[row],
            self.runner.y_wp[row - 1],
        )

        # theta, v_trans のインデックス入れ替え
        self._swap_theta_v_indices(row, row - 1)

        self.update_waypoint_table()
        self.table.selectRow(row - 1)
        self.draw_waypoints_only()

    def on_down_clicked(self) -> None:
        """選択された waypoint を一つ下に移動する"""
        row = self.table.currentRow()
        if row < 0 or row >= len(self.runner.x_wp) - 1:
            self.log_action("waypoint下へ: 移動できません")
            return
        self._push_waypoint_undo()
        self.log_action(f"waypoint下へ: row={row} -> {row+1}")

        # x, y の入れ替え
        self.runner.x_wp[row + 1], self.runner.x_wp[row] = (
            self.runner.x_wp[row],
            self.runner.x_wp[row + 1],
        )
        self.runner.y_wp[row + 1], self.runner.y_wp[row] = (
            self.runner.y_wp[row],
            self.runner.y_wp[row + 1],
        )

        # theta, v_trans のインデックス入れ替え
        self._swap_theta_v_indices(row, row + 1)

        self.update_waypoint_table()
        self.table.selectRow(row + 1)
        self.draw_waypoints_only()

    def update_waypoint_table(self) -> None:
        """runner 内の waypoint 情報をテーブルに反映する"""
        x_wp, y_wp, theta_wp, v_trans_wp = self.runner.get_waypoints()

        n = len(x_wp)
        self.table.blockSignals(True)
        try:
            self.table.setRowCount(n)

            for i in range(n):
                idx_item = QTableWidgetItem(str(i))
                x_item = QTableWidgetItem(self._format_waypoint_value(x_wp[i]))
                y_item = QTableWidgetItem(self._format_waypoint_value(y_wp[i]))
                theta_val = theta_wp[i]
                v_val = v_trans_wp[i]
                theta_item = QTableWidgetItem(
                    "" if not math.isfinite(theta_val) else self._format_waypoint_value(theta_val)
                )
                v_item = QTableWidgetItem(
                    "" if not math.isfinite(v_val) else self._format_waypoint_value(v_val)
                )

                self.table.setItem(i, 0, idx_item)
                self.table.setItem(i, 1, x_item)
                self.table.setItem(i, 2, y_item)
                self.table.setItem(i, 3, theta_item)
                self.table.setItem(i, 4, v_item)
        finally:
            self.table.blockSignals(False)

    def _format_waypoint_value(self, value: float) -> str:
        """waypoint テーブル向けに double 相当の桁数で表示する"""
        if value == 0.0:
            value = 0.0
        return format(value, f".{self.WAYPOINT_DISPLAY_SIG_DIGITS}g")

    def on_waypoint_item_changed(self, item: QTableWidgetItem) -> None:
        """waypoint テーブルの編集を runner とプロットへ即時反映する"""
        if item.column() not in [1, 2, 3, 4]:
            return
        selected_row = self.table.currentRow()
        # runner 反映前の状態を undo に積む（runner は編集前の値を保持している）
        self._push_waypoint_undo()
        self.update_runner_from_table(refresh_table=False)
        if 0 <= selected_row < self.table.rowCount():
            self.table.selectRow(selected_row)
        self.draw_waypoints_only()

    def on_canvas_clicked(self, event) -> None:
        """グラフクリックで waypoint を追加する"""
        if event.inaxes is not self.ax:
            return

        if event.xdata is None or event.ydata is None:
            return

        x_val = float(event.xdata)
        y_val = float(event.ydata)

        selected_row = self.table.currentRow()
        mode = getattr(self, "_waypoint_click_mode", "edit")

        if mode in ["insert_above", "insert_below"] and not (
            0 <= selected_row < len(self.runner.x_wp)
        ):
            # 挿入モードだが選択がない場合は末尾に追加
            mode = "edit"

        if mode == "insert_above":
            self._push_waypoint_undo()
            insert_index = selected_row
            self.log_action(
                f"waypoint挿入(上): insert={insert_index}, x={x_val:.3f}, y={y_val:.3f} (zone={self.runner.zone})"
            )
            self._insert_waypoint_at(insert_index, x_val, y_val)
            return

        if mode == "insert_below":
            self._push_waypoint_undo()
            insert_index = selected_row + 1
            self.log_action(
                f"waypoint挿入(下): insert={insert_index}, x={x_val:.3f}, y={y_val:.3f} (zone={self.runner.zone})"
            )
            self._insert_waypoint_at(insert_index, x_val, y_val)
            return

        # 編集モード: 選択があれば上書き、なければ末尾に追加
        if 0 <= selected_row < len(self.runner.x_wp):
            self._push_waypoint_undo()
            self.runner.x_wp[selected_row] = x_val
            self.runner.y_wp[selected_row] = y_val
            self.log_action(
                f"waypoint上書き: row={selected_row}, x={x_val:.3f}, y={y_val:.3f} (zone={self.runner.zone})"
            )
            self.update_waypoint_table()
            if 0 <= selected_row < self.table.rowCount():
                self.table.selectRow(selected_row)
            self.draw_waypoints_only()
            return

        self._push_waypoint_undo()
        insert_index = len(self.runner.x_wp)
        self.log_action(
            f"waypoint追加: insert={insert_index}, x={x_val:.3f}, y={y_val:.3f} (zone={self.runner.zone})"
        )
        self._insert_waypoint_at(insert_index, x_val, y_val)

    def _get_waypoint_snapshot(self) -> dict:
        return {
            "x_wp": list(self.runner.x_wp),
            "y_wp": list(self.runner.y_wp),
            "theta_wp": list(self.runner.theta_wp),
            "v_trans_wp": list(self.runner.v_trans_wp),
            "selected_row": int(self.table.currentRow()),
        }

    def _restore_waypoint_snapshot(self, snap: dict) -> None:
        self.runner.x_wp = list(snap.get("x_wp", []))
        self.runner.y_wp = list(snap.get("y_wp", []))
        self.runner.theta_wp = list(snap.get("theta_wp", []))
        self.runner.v_trans_wp = list(snap.get("v_trans_wp", []))
        self.update_waypoint_table()
        row = int(snap.get("selected_row", -1))
        if 0 <= row < self.table.rowCount():
            self.table.selectRow(row)
        self.draw_waypoints_only()

    def _push_waypoint_undo(self) -> None:
        snap = self._get_waypoint_snapshot()
        if self._waypoint_undo_stack:
            prev = self._waypoint_undo_stack[-1]
            if (
                prev.get("x_wp") == snap.get("x_wp")
                and prev.get("y_wp") == snap.get("y_wp")
                and prev.get("theta_wp") == snap.get("theta_wp")
                and prev.get("v_trans_wp") == snap.get("v_trans_wp")
            ):
                return
        self._waypoint_undo_stack.append(snap)
        # undo が積まれたら redo は無効
        self._waypoint_redo_stack.clear()
        # 無限に増えないように上限を設ける
        max_hist = 200
        if len(self._waypoint_undo_stack) > max_hist:
            self._waypoint_undo_stack = self._waypoint_undo_stack[-max_hist:]

    def on_undo_clicked(self) -> None:
        """waypoint 操作を1つ戻す"""
        if not self._waypoint_undo_stack:
            self.log_action("戻る: 履歴がありません")
            return
        cur = self._get_waypoint_snapshot()
        snap = self._waypoint_undo_stack.pop()
        self._waypoint_redo_stack.append(cur)
        self.log_action("戻る: 実行")
        self._restore_waypoint_snapshot(snap)

    def on_redo_clicked(self) -> None:
        """waypoint 操作を1つ進める"""
        if not self._waypoint_redo_stack:
            self.log_action("進む: 履歴がありません")
            return
        cur = self._get_waypoint_snapshot()
        snap = self._waypoint_redo_stack.pop()
        self._waypoint_undo_stack.append(cur)
        self.log_action("進む: 実行")
        self._restore_waypoint_snapshot(snap)

    def update_runner_from_table(self, refresh_table: bool = True) -> None:
        """テーブル内容を runner の waypoint に反映する"""
        row_count = self.table.rowCount()

        new_x: list[float] = []
        new_y: list[float] = []
        new_theta: list[float] = []
        new_v: list[float] = []

        for row in range(row_count):
            x_item = self.table.item(row, 1)
            y_item = self.table.item(row, 2)
            theta_item = self.table.item(row, 3)
            v_item = self.table.item(row, 4)

            if x_item is None or y_item is None:
                continue

            try:
                x_val = float(x_item.text())
                y_val = float(y_item.text())
            except ValueError:
                continue

            new_x.append(x_val)
            new_y.append(y_val)

            if theta_item is not None and theta_item.text().strip() != "":
                try:
                    new_theta.append(float(theta_item.text()))
                except ValueError:
                    new_theta.append(math.inf)
            else:
                new_theta.append(math.inf)

            if v_item is not None and v_item.text().strip() != "":
                try:
                    new_v.append(float(v_item.text()))
                except ValueError:
                    new_v.append(math.inf)
            else:
                new_v.append(math.inf)

        self.runner.x_wp = new_x
        self.runner.y_wp = new_y
        self.runner.theta_wp = new_theta
        self.runner.v_trans_wp = new_v

        if refresh_table:
            # runner の内容を基準にテーブルを整える
            self.update_waypoint_table()

    def update_param_table(self) -> None:
        """runner が持つ各種パラメータを表示する"""
        params = [
            ("zone", self.runner.zone),
            ("dt", f"{self.runner.dt:.3f}"),
            ("v_max", f"{self.runner.v_max:.3f}"),
            ("a_max", f"{self.runner.a_max:.3f}"),
            ("j_max", f"{self.runner.j_max:.3f}"),
            ("omega_max", f"{self.runner.omega_max:.3f}"),
            ("robot_dt", f"{self.runner.robot_dt:.3f}"),
        ]

        self.param_table.blockSignals(True)
        try:
            self.param_table.setRowCount(len(params))
            for row, (name, value) in enumerate(params):
                name_item = QTableWidgetItem(str(name))
                value_item = QTableWidgetItem(str(value))
                self.param_table.setItem(row, 0, name_item)
                self.param_table.setItem(row, 1, value_item)
        finally:
            self.param_table.blockSignals(False)

    def on_param_item_changed(self, item: QTableWidgetItem) -> None:
        """パラメータ編集（zone 等）を即時 runner と描画に反映する"""
        if item.column() != 1:
            return
        row = item.row()
        name_item = self.param_table.item(row, 0)
        if name_item is None:
            return
        name = name_item.text().strip()
        value_text = item.text().strip()

        if name != "zone":
            return

        value_text = value_text.lower()
        if value_text not in ["red", "blue"]:
            return

        prev_zone = self.runner.zone
        if value_text == prev_zone:
            return

        self.runner.zone = value_text
        self.log_action(f"zone変更: {prev_zone} -> {self.runner.zone} (即時反映)")
        self.reset_view()
        self.draw_waypoints_only()

    def update_runner_prefixes_from_edits(self) -> None:
        """ファイル名ベースの入力値を runner に反映する"""
        input_prefix = self.input_prefix_edit.text().strip()
        output_prefix = self.output_prefix_edit.text().strip()

        if not input_prefix:
            return

        self.runner.input_prefix = input_prefix
        # 出力が空なら入力と同じにする
        self.runner.output_prefix = output_prefix if output_prefix else input_prefix

    def update_runner_params_from_table(self) -> None:
        """パラメータテーブルの内容を runner に反映する"""
        row_count = self.param_table.rowCount()
        for row in range(row_count):
            name_item = self.param_table.item(row, 0)
            value_item = self.param_table.item(row, 1)
            if name_item is None or value_item is None:
                continue
            name = name_item.text()
            value_text = value_item.text()

            try:
                if name == "zone":
                    if value_text in ["red", "blue"]:
                        self.runner.zone = value_text
                elif name == "dt":
                    self.runner.dt = float(value_text)
                elif name == "v_max":
                    self.runner.v_max = float(value_text)
                elif name == "a_max":
                    self.runner.a_max = float(value_text)
                elif name == "j_max":
                    self.runner.j_max = float(value_text)
                elif name == "omega_max":
                    self.runner.omega_max = float(value_text)
                elif name == "robot_dt":
                    self.runner.robot_dt = float(value_text)
            except ValueError:
                # 数値変換に失敗した場合はそのパラメータを無視
                continue

    def _swap_theta_v_indices(self, a: int, b: int) -> None:
        """theta_wp / v_trans_wp 内の index a と b を入れ替える"""
        self.runner.theta_wp[a], self.runner.theta_wp[b] = (
            self.runner.theta_wp[b],
            self.runner.theta_wp[a],
        )
        self.runner.v_trans_wp[a], self.runner.v_trans_wp[b] = (
            self.runner.v_trans_wp[b],
            self.runner.v_trans_wp[a],
        )

    def on_waypoint_selection_changed(self) -> None:
        """waypoint テーブルの選択変更時にハイライトを更新"""
        self.draw_waypoints_only()

    def draw_waypoints_only(self) -> None:
        """フィールドと waypoint のみを描画する"""
        self.stop_animation(log_message=None)
        self.robot_preview_patches = []
        # すでにズーム・パンが設定されている場合は、現在の表示範囲を保持して再描画する
        if self.base_xlim is None or self.base_ylim is None:
            # 初回: 基準表示範囲を設定
            self.fig.clear()
            self.ax = self.fig.add_subplot(111)
            self.runner.fig = self.fig
            self.runner.ax = self.ax

            self.runner._plot_field()
            self.runner._set_axis_range()
            # ズーム基準範囲を更新
            self.base_xlim = self.ax.get_xlim()
            self.base_ylim = self.ax.get_ylim()
        else:
            # 現在の表示範囲を保持したまま再描画
            cur_xlim = self.ax.get_xlim()
            cur_ylim = self.ax.get_ylim()

            self.ax.cla()
            self.runner.ax = self.ax
            self.runner._plot_field()

            # 以前の表示範囲を復元して、ズーム・パン状態を維持
            self.ax.set_xlim(cur_xlim)
            self.ax.set_ylim(cur_ylim)
            self.ax.set_aspect("equal", adjustable="box")

        # waypoint 座標は zone によらず CSV の値をそのまま描画
        x_wp, y_wp, *_ = self.runner._get_zone_adjusted_waypoints()
        if x_wp and y_wp:
            selected_row = self.table.currentRow()

            # 選択されている waypoint 以外を通常色で描画
            if 0 <= selected_row < len(x_wp):
                other_x = [x_wp[i] for i in range(len(x_wp)) if i != selected_row]
                other_y = [y_wp[i] for i in range(len(x_wp)) if i != selected_row]
            else:
                other_x = x_wp
                other_y = y_wp
                selected_row = -1

            if other_x:
                self.ax.scatter(
                    other_x,
                    other_y,
                    color="red",
                    marker="x",
                    s=40,
                    label="Waypoints",
                    zorder=5,
                )

            # 選択中の waypoint を強調表示
            if selected_row >= 0:
                self.ax.scatter(
                    [x_wp[selected_row]],
                    [y_wp[selected_row]],
                    color="yellow",
                    edgecolors="black",
                    marker="o",
                    s=80,
                    label="Selected",
                    zorder=10,
                )

            self.ax.legend()

        self.canvas.draw()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
