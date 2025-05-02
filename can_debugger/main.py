import math
import sys
import time
import struct

import json

from PySide6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QSizePolicy, QCheckBox, QDoubleSpinBox, QSpacerItem
from PySide6.QtCore import Qt, QThread, QObject, Signal, Slot, QTimer, QPointF
from PySide6.QtCharts import QChartView, QChart, QValueAxis, QLineSeries


from can import CANUSB, CANUSB_SPEED

class PlotterWidget(QWidget):
    def __init__(self, parent):
        super().__init__(parent)

        self.channels = []

        self.setup_ui()    
        self.setup_chart()    
        self.setup_scaling()


    def setup_ui(self):
        self.setLayout(QHBoxLayout())
        self.chart = QChart()
        self.view = QChartView(self)
        self.view.setChart(self.chart)
        self.layout().addWidget(self.view)

        self.setSizePolicy(QSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding))
        self.view.setSizePolicy(QSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding))

    def setup_chart(self):
        self.xAxis = QValueAxis()
        self.yAxis = QValueAxis()

        self.chart.addAxis(self.xAxis, Qt.AlignBottom)
        self.chart.addAxis(self.yAxis, Qt.AlignLeft)

        self.view.setRubberBand(QChartView.RectangleRubberBand)

    def setup_scaling(self):
        self.scaling_widget = QWidget()
        self.layout().addWidget(self.scaling_widget)
        self.scaling_layout = QVBoxLayout()

        self.auto_x_check = QCheckBox("Auto Time", self.scaling_widget)
        self.scaling_layout.addWidget(self.auto_x_check)
        self.auto_x_check.setChecked(True)
        self.auto_y_check = QCheckBox("Auto Value", self.scaling_widget)
        self.scaling_layout.addWidget(self.auto_y_check)
        self.auto_y_check.setChecked(True)
        self.time_spin = QDoubleSpinBox(self.scaling_widget)
        self.scaling_layout.addWidget(self.time_spin)

        self.scaling_spacer = QSpacerItem(1, 1, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)
        self.scaling_layout.addItem(self.scaling_spacer)

        self.scaling_widget.setLayout(self.scaling_layout)

    def range(self):
        if self.auto_x_check.isChecked():
            min_time = min((channel.min_time for channel in self.channels))
            max_time = max((channel.max_time for channel in self.channels))
            if min_time - max_time == 0:
                max_time = min_time + 1e-10
            self.xAxis.setRange(min_time, max_time)

        if self.auto_y_check.isChecked():
            min_value = min((channel.min_value for channel in self.channels))
            max_value = max((channel.max_value for channel in self.channels))
            if min_value - max_value == 0:
                max_value = min_value + 1e-10
            self.yAxis.setRange(min_value, max_value)

        if self.time_spin.value() > 0:
            max_time = max((channel.max_time for channel in self.channels))
            min_time = max_time - self.time_spin.value()
            self.xAxis.setRange(min_time, max_time)

class Channel():
    def __init__(self, plotter: PlotterWidget):
        self.plotter = plotter
        self.plotter.channels.append(self)

        self.series = QLineSeries()
        self.series.useOpenGL()
        self.plotter.chart.addSeries(self.series)
        self.series.attachAxis(self.plotter.xAxis)
        self.series.attachAxis(self.plotter.yAxis)

        self.points: list[QPointF] = []
        self.min_time = 1e20
        self.max_time = -1e20
        self.min_value = 1e20
        self.max_value = -1e20

    def add_point(self, time, value):
        self.points.append(QPointF(time,value))
        if len(self.points) > 20000:
            self.points.pop(0)

    def update_plot(self):
        if self.points:
            self.series.replace(self.points)

            self.min_time = min((p.x() for p in self.points))
            self.max_time = max((p.x() for p in self.points))
            self.min_value = min((p.y() for p in self.points))
            self.max_value = max((p.y() for p in self.points))
            self.plotter.range()

        # print(len(self.points))


class MainWindow(QMainWindow):
    start_listening = Signal()

    def __init__(self):
        super().__init__()

        self.channels: dict[int, Channel] = {}

        self.start_listener()
        self.start_display_update()


        self.plotters = []

        self.widget = QWidget()
        self.setCentralWidget(self.widget)
        self.layout = QVBoxLayout(self.widget)

        self.parse_config()


    def new_channel(self, id: int, label: str, widget: PlotterWidget):
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

    def parse_config(self):
        file = open('config.json')
        config = json.load(file)

        for plot in config["plot"]:
            print(plot)
            w = PlotterWidget(self)
            self.layout.addWidget(w)
            self.plotters.append(w)
            for channel in plot["channel"]:
                print(channel)
                self.new_channel(channel["id"],channel["label"],w)


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
            # print(frame.id)
            if len(frame.data)!= 4:
                print("MALFORMED FRAME----------------------------")
                continue
            value, = struct.unpack("<i", frame.data)
            timestamp = float(time.time()-self.start_time_us)
            self.new_data.emit(frame.id, timestamp, value)



if __name__ == "__main__":

 

    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec())