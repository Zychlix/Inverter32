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
aline, = ax.plot(BUFFER_SIZE * [0], label="1")
bline, = ax.plot(BUFFER_SIZE * [0], label="2")
plt.legend()
# cline, = ax.plot(BUFFER_SIZE * [0])

while True:
    line = None
    while line != "START":
        line = ser.readline().decode('ASCII').strip()
        print(line)

    data1 = []
    data2 = []
    try:
        for i in range(BUFFER_SIZE):
            line = ser.readline().decode('ASCII').strip()
            a, b = line.split(' ')
            data1.append(float(a))
            data2.append(float(b))
    except ValueError as e:
        print(e)
        continue

    data1 = np.array(data1)
    data2 = np.array(data2)
    # data3 = -data1 - data2

    d1_std = np.std(data1)
    d2_std = np.std(data2)

    aline.set_ydata(data1)
    bline.set_ydata(data2)
    # cline.set_ydata(data3)
    ax.relim()
    ax.autoscale_view()
    fig.canvas.draw()
    fig.canvas.flush_events()
