import matplotlib.pyplot as plt
from multiprocessing import Process, Manager, Queue
import numpy as np
from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph as pg

from data_utils import *

FIELD_LENGTH = 15.98  # m
FIELD_WIDTH = 8.21  # m


class CenteredArrowItem(pg.ArrowItem):
    def paint(self, p, *args):
        p.translate(-self.boundingRect().center())
        pg.ArrowItem.paint(self, p, *args)


class Plotter:
    def __init__(self, num_particles):
        self.num_particles = num_particles

    def run(self):
        app = pg.mkQApp()

        self.win = pg.GraphicsWindow(title="QuixPF")
        self.win.resize(1000, 500)

        # Create 2D plotting area
        plot = self.win.addPlot()
        plot.setAspectLocked()

        # Show field
        img = pg.ImageItem()
        img.setImage(np.rot90(plt.imread('2021_field.png'), k=3))
        img.setRect(-FIELD_LENGTH * 0.5, -FIELD_WIDTH * 0.5, FIELD_LENGTH, FIELD_WIDTH)
        plot.addItem(img)

        # Initialize scatter plot for particles
        self.scatter = pg.ScatterPlotItem(size=self.num_particles)
        self.scatter.setData(size=1, pen=pg.mkPen(None))
        plot.addItem(self.scatter)

        # Initialize arrow for best estimate
        self.arrow = CenteredArrowItem()
        plot.addItem(self.arrow)

        timer = QtCore.QTimer()
        timer.timeout.connect(self._update)
        timer.start(1000.0/60.0)

        app.exec()

    def start(self):
        self.q = Queue()
        self.p = Process(target=self.run)
        self.p.start()
        return self.q

    def join(self):
        return self.p.join()

    def _update(self):
        # Fast fowrard to the latest element
        latest = None
        while not self.q.empty():
            latest = self.q.get()

        if latest is not None:
            particles, best_estimate, has_vision, facing_our_goal = latest

            # Particles
            NUM_TO_PLOT = 1000
            self.scatter.setData(
                x=particles[:NUM_TO_PLOT, 0, 3],
                y=particles[:NUM_TO_PLOT, 1, 3],
                brush=('g' if facing_our_goal else 'b') if has_vision else 'r',
            )

            x, y, theta = best_estimate
            self.arrow.setPos(x, y)
            self.arrow.setStyle(angle=-np.rad2deg(theta) + 180)
