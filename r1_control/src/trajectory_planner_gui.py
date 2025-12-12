import sys

from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from PyQt6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget

from run_trajectory_planner import get_fig

class MplCanvas(FigureCanvasQTAgg):
    """matplotlib Figure を PyQt6 に埋め込むキャンバスクラス"""

    def __init__(self, parent=None):
        fig = Figure(figsize=(5, 4), dpi=100)
        self.ax = fig.add_subplot(111)
        super().__init__(fig)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        # 中央ウィジェット
        widget = QWidget()
        layout = QVBoxLayout(widget)

        # matplotlib キャンバス生成
        self.canvas = MplCanvas(self)

        # グラフ描画
        x = [0, 1, 2, 3, 4]
        y = [0, 1, 4, 9, 16]
        self.canvas.ax.plot(x, y)
        self.canvas.ax.set_title("PyQt6 + Matplotlib Example")

        layout.addWidget(self.canvas)
        self.setCentralWidget(widget)

        self.setWindowTitle("PyQt6 Matplotlib Demo")
        self.resize(600, 400)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
