import serial
import matplotlib.pyplot as plt
import sys
import numpy as np


def on_close(_):
    sys.exit(0)


BUFFER_SIZE = 512

ser = serial.Serial('/dev/ttyACM0', 115200)
plt.ion()
fig = plt.figure()
fig.canvas.mpl_connect('close_event', on_close)
ax = fig.add_subplot(111)
aline, = ax.plot(BUFFER_SIZE * [0])
bline, = ax.plot(BUFFER_SIZE * [0])

while True:
    line = None
    while line != "START":
        line = ser.readline().decode('ASCII').strip()
    print("receeived start")

    data1 = []
    data2 = []
    try:
        for i in range(BUFFER_SIZE):
            line = ser.readline().decode('ASCII').strip()
            print(line)
            a, b = line.split(' ')
            data1.append(float(a))
            data2.append(float(b))
    except ValueError as e:
        print(e)
        continue

    aline.set_ydata(data1)
    bline.set_ydata(data2)
    ax.relim()
    ax.autoscale_view()
    fig.canvas.draw()
    fig.canvas.flush_events()
