import math
import sys
import time
import struct

from PySide6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout
from PySide6.QtCore import Qt, QThread, QObject, Signal, Slot, QTimer, QPointF
from PySide6.QtCharts import QChartView, QChart, QValueAxis, QLineSeries


from can import CANUSB, CANUSB_SPEED

class PlotterWidget(QChartView):
    def __init__(self, parent):
        super().__init__(parent)
        self.setChart(QChart())

        self.xAxis = QValueAxis()
        self.yAxis = QValueAxis()

        self.chart().addAxis(self.xAxis, Qt.AlignBottom)
        self.chart().addAxis(self.yAxis, Qt.AlignLeft)

        self.setRubberBand(QChartView.RectangleRubberBand)

        self.minimum = 1e20
        self.maximum = -1e20
        self.max_time = 0


    def range(self, minimum, maximum, max_time):
        self.minimum = min(self.minimum, minimum)
        self.maximum = max(self.maximum, maximum)
        self.max_time = max(self.max_time, max_time)

        self.xAxis.setRange(self.max_time-20,self.max_time)
        self.yAxis.setRange(self.minimum, self.maximum)

class Channel():
    def __init__(self, plotter: PlotterWidget):
        self.plotter = plotter

        self.series = QLineSeries()
        self.series.useOpenGL()
        self.plotter.chart().addSeries(self.series)
        self.series.attachAxis(self.plotter.xAxis)
        self.series.attachAxis(self.plotter.yAxis)

        self.points: list[QPointF] = []
        self.minimum = 1e20
        self.maximum = -1e20
        self.max_time = 0

    def add_point(self, time, value):
        self.points.append(QPointF(time,value))
        self.minimum = min(value, self.minimum)
        self.maximum = max(value, self.maximum)
        self.max_time = time
        if len(self.points) > 20000:
            self.points.pop(0)

    def update_plot(self):
        self.series.replace(self.points)
        self.plotter.range(self.minimum, self.maximum,  self.max_time)


class MainWindow(QMainWindow):
    start_listening = Signal()

    def __init__(self):
        super().__init__()

        self.channels: dict[int, Channel] = {}

        self.start_listener()
        self.start_display_update()

        self.w1 = PlotterWidget(self)
        self.setCentralWidget(self.w1)
        self.w2 = PlotterWidget(self)

        # self.layout = QVBoxLayout()
        # self.layout.addWidget(self.w1)
        # self.layout.addWidget(self.w2)
        # self.setLayout(self.layout)

        self.new_channel(0x100, self.w1)
        self.new_channel(0x101, self.w2)

    def new_channel(self, id: int, widget: PlotterWidget):
        self.channels[id] = Channel(widget)

    def start_listener(self):
        self.listener_thread = QThread()
        self.listener = CanListener()
        self.listener.moveToThread(self.listener_thread)
        self.listener.new_data.connect(self.new_data)
        self.listener_thread.start()

        self.start_listening.connect(self.listener.listen)
        self.start_listening.emit()

    def start_display_update(self):
        self.timer = QTimer(self)
        self.timer.setInterval(20)
        self.timer.timeout.connect(self.update_plotters)
        self.timer.start()

    def update_plotters(self):
        for channel in self.channels.values():
            channel.update_plot()

    @Slot(int, float, float)
    def new_data(self, channel, timestamp, value):
        channel = self.channels.get(channel, None)
        if channel is not None:
            channel.add_point(timestamp, value)

class CanListener(QObject):
    new_data = Signal(int, float, float)

    def __init__(self):
        super().__init__()
        self.can = CANUSB("/dev/ttyUSB0", CANUSB_SPEED.SPEED_500000)
        self.start_time_us = time.time()
        
    @Slot()
    def listen(self):
        while True:
            frame = self.can.get_frame()
            value, = struct.unpack("<i", frame.data)
            timestamp = float(time.time()-self.start_time_us)
            self.new_data.emit(frame.id, timestamp, value)



if __name__ == "__main__":

    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec())