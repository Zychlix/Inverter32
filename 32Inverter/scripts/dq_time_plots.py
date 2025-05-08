import time

import serial
import numpy as np
import matplotlib.pyplot as plt

class CommunicationError(Exception):
    def __init__(self, received: str):
        super().__init__(self, f'Unexpectedly received: {received}')

class SerialWrap(serial.Serial):
    def write(self, data, *args, **kwargs):
        print(f'TX: {data}')
        super().write(data, *args, **kwargs)

    def readline(self, *args, **kwargs):
        result = super().readline(*args, **kwargs)
        print(f'RX: {result}')
        return result


class InverterAPI:
    def __init__(self, port='/dev/ttyACM0', baud=115200):
        self.ser = serial.Serial(port, baud, timeout=2)

    def set_ab(self, current: float, fi: float):
        resp = self.query(f'ab {current:.1f} {fi:.3f}')
        if not resp.startswith('A'):
            raise CommunicationError(resp)
    
    def set_vf(self, frequency: float, current_d: float, current_q: float):
        resp = self.query(f'vf  {frequency:.3f} {current_d:.1f} {current_q:.1f}')
        if not resp.startswith('OK'):
            raise CommunicationError(resp)
    def set_plot_channel(self, channel:int):
        resp = self.query(f'channel  {channel:.1f}')
        if not resp.startswith('OK'):
            raise CommunicationError(resp)
        
    def set_plot(self, points):
        resp = self.query(f'plot')
        a = 1

        while a:
            a = self.ser.readline().decode('utf-8').strip()
            points.append(a)
            if(a.startswith('O') ):
                return

    def pi(self, kp: float, ki: float):
        resp = self.query(f'pi a {kp} {ki}')
        if not resp == 'ok':
            raise CommunicationError(resp)

    def res_angle(self):
        resp = self.query(f'res_angle')
        fi = float(resp)
        return fi

    def ping(self):
        resp = self.query('ping')
        print(resp)
    def query(self, command: str) -> str:
        RETRY_COUNT = 3
        for i in range(RETRY_COUNT):
            self.ser.write((command + '\n').encode('utf-8'))
            resp = self.ser.readline().decode('utf-8').strip()

            if resp == '':
                print("Timeout")
            elif resp == 'Unknown command!':
                print("Unknown command")
            else:
                return resp

            time.sleep(0.5)

        raise Exception("Query failed")


inv = InverterAPI()
time.sleep(2)
inv.ping()

for i in range(5):
    try:
        inv.pi(0, 1)
    except Exception as e:
        print(e)
        time.sleep(0.5)
    else:
        break


#Inverter is responsive. Can proceed.

unwrapping_offset = 0

N_POINTS = 2
FREQ_LOW = 10
FREQ_HIGH = 1000
freq = np.linspace(0, 1000, N_POINTS)
fi_mechanical = []

current = 5

inv.set_plot_channel(0) #Y channel

points = []

inv.set_plot(points)

print(points)


for freq_ele in freq:
    inv.set_vf(freq, 0,current)
   
    print(f'{fi_ele:.3f} -> {fi_mech:.3f}')

b, a = np.polyfit(fi_electrical, fi_mechanical, deg=1)
print(a, b)



inv.set_ab(0, 0)

plt.plot(fi_electrical, a + b * fi_electrical, color="k", lw=2.5)
plt.plot(fi_electrical, fi_mechanical, 'o')
plt.show()