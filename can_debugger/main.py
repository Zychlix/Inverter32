import math
import sys

from PySide6.QtWidgets import QWidget, QApplication
from PySide6.QtCore import QPoint, QRect, QTimer, Qt, QLine
from PySide6.QtGui import QPainter, QPointList

import PySide6.QtGui as pqp

from usb_can_adapter_v1 import UsbCanAdapter
import can_debug_interface

class cdi_channel:
    def __init__(self, id, interface):
        self.id = id
        self.cdi = interface
        self.color = pqp.QColor('black')
        self._points = QPointList()
        self._points.reserve(WIDTH)

        self._x = 0
        self._delta_x = 1
        self._half_height = HEIGHT / 2
        self._factor = 1/Y_SCALE * self._half_height
        self.color = pqp.QColor('red')

    def next_point(self):
        # self.cdi.update()
        # result = self._half_height + self.cdi.channels[0].value
        # print('self id: '+str(self.id))
        result = self._half_height - self._factor * self.cdi.channels[self.id].value
        # print(self.cdi.channels[0].value)

        self._x += self._delta_x
        return result

#TODO assert channel count

WIDTH = 2000
HEIGHT = 1000

Y_SCALE = 100


class PlotWidget(QWidget):
    """Illustrates the use of opaque containers. QPointList
       wraps a C++ QList<QPoint> directly, removing the need to convert
       a Python list in each call to QPainter.drawPolyline()."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._timer = QTimer(self)
        self._timer.setInterval(10)
        self._timer.timeout.connect(self.shift)


        self._data_timer = QTimer(self)
        self._data_timer.setInterval(3)

        self.channels = []


        self.uca = UsbCanAdapter()
        self.uca.set_port('/dev/ttyUSB0')
        self.uca.adapter_init()
        self.uca.command_settings(speed=500000)

        self.cdi = can_debug_interface.cdi_interface(self.uca)

        #declare channels
        self.cdi.add_channel(0x100,int,0)

        self.cdi.add_channel(0x101,int,1)

        
        self._data_timer.timeout.connect(self.dataPoll)



        #Create traces
        for i in range(len(self.cdi.channels)):
            self.channels.append(cdi_channel(i,self.cdi))


        for channel in self.channels:
            for i in range(WIDTH):
                channel._points.append(QPoint(i, channel.next_point()))

        self.channels[1].color= pqp.QColor('green')

        self.setFixedSize(WIDTH, HEIGHT)

        self._data_timer.start()
        self._timer.start()

        self.pen = pqp.QPen()



    def shift(self):
        for channel in self.channels:
            last_x = channel._points[WIDTH - 1].x()
            channel._points.pop_front()
            channel._points.append(QPoint(last_x + 1, channel.next_point()))
        self.update()

    def paintEvent(self, event):
        with QPainter(self) as painter:
            rect = QRect(QPoint(0, 0), self.size())
            painter.fillRect(rect, Qt.GlobalColor.white)
            painter.translate(-self.channels[0]._points[0].x(), 0) #fixed!
            # self.verticalReticle(1,1)
            for channel in self.channels:    
                # print("channel: " + str(channel.id))
                # painter.translate(0, channel.id*50)
                painter.setPen(self.pen)
                self.pen.setColor(channel.color)

                painter.drawPolyline(channel._points)
                painter.drawLine(QLine(0,0,1000,1000))
        

    def verticalReticle(self, intervals, scale):
        pen = pqp.QPen()
        with QPainter(self) as painter:
            painter.setPen(self.pen)
           
        pass

    def dataPoll(self):
        self.cdi.update()
        pass
        


if __name__ == "__main__":

    app = QApplication(sys.argv)

    w = PlotWidget()
    w.show()
    sys.exit(app.exec())