import sys
import numpy as np
import pandas as pd

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QFileDialog, QLabel, QSpinBox
)

from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qtagg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure


class Mpl3DCanvas(FigureCanvas):
    def __init__(self):
        self.fig = Figure()
        self.ax = self.fig.add_subplot(111, projection="3d")
        super().__init__(self.fig)

    def plot_xyz(self, x, y, z, title="3D XYZ Trajectory"):
        self.ax.clear()

        # trajectory
        self.ax.plot(x, y, z)
        self.ax.scatter(x, y, z, s=5)

        # START / END markers
        if len(x) > 0:
            xs, ys, zs = x[0], y[0], z[0]
            xe, ye, ze = x[-1], y[-1], z[-1]

            self.ax.scatter([xs], [ys], [zs], s=150, marker="o")
            self.ax.text(xs, ys, zs, " START")

            self.ax.scatter([xe], [ye], [ze], s=150, marker="X")
            self.ax.text(xe, ye, ze, " END")

        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")
        self.ax.set_title(title)

        self.draw()


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("3D XYZ Viewer")
        self.resize(1200, 800)

        self.df = None

        # Buttons
        self.btn_open = QPushButton("Open CSV")
        self.btn_plot = QPushButton("Plot")
        self.btn_plot.setEnabled(False)

        self.downsample = QSpinBox()
        self.downsample.setRange(1, 10000)
        self.downsample.setValue(1)

        # Small one-line info label
        self.info = QLabel("No file loaded")
        self.info.setMaximumHeight(20)

        # Canvas
        self.canvas = Mpl3DCanvas()
        self.toolbar = NavigationToolbar(self.canvas, self)

        # ===== Layout =====
        top_bar = QWidget()
        top_layout = QHBoxLayout(top_bar)
        top_layout.setContentsMargins(5, 5, 5, 5)
        top_layout.setSpacing(10)

        top_layout.addWidget(self.btn_open)
        top_layout.addWidget(QLabel("Downsample:"))
        top_layout.addWidget(self.downsample)
        top_layout.addWidget(self.btn_plot)
        top_layout.addStretch()
        top_layout.addWidget(self.info)

        central = QWidget()
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(5, 5, 5, 5)
        main_layout.setSpacing(5)

        main_layout.addWidget(top_bar)
        main_layout.addWidget(self.toolbar)
        main_layout.addWidget(self.canvas, stretch=1)  # <- Grafik dapat space maksimal

        self.setCentralWidget(central)

        # Signals
        self.btn_open.clicked.connect(self.open_csv)
        self.btn_plot.clicked.connect(self.plot_3d)

    def open_csv(self):
        path, _ = QFileDialog.getOpenFileName(self, "Open CSV", "", "CSV Files (*.csv)")
        if not path:
            return

        try:
            df = pd.read_csv(path)
            df.columns = [c.strip() for c in df.columns]

            if not {"X", "Y", "Z"}.issubset(df.columns):
                self.info.setText("Kolom harus X,Y,Z")
                return

            for c in ["X", "Y", "Z"]:
                df[c] = pd.to_numeric(df[c], errors="coerce")

            df = df.dropna(subset=["X", "Y", "Z"])

            self.df = df
            self.info.setText(f"{path.split('/')[-1]}  |  Rows: {len(df)}")

            self.btn_plot.setEnabled(True)
            self.plot_3d()

        except Exception as e:
            self.info.setText(f"Error: {e}")

    def plot_3d(self):
        if self.df is None:
            return

        step = int(self.downsample.value())
        d = self.df.iloc[::step]

        x = d["X"].to_numpy()
        y = d["Y"].to_numpy()
        z = d["Z"].to_numpy()

        self.canvas.plot_xyz(x, y, z, title=f"3D XYZ (step={step})")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())