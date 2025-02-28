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
    def __init__(self, port='/dev/ttyACM0', baud=9600):
        self.ser = SerialWrap(port, baud, timeout=2)

    def set_ab(self, current: float, fi: float):
        resp = self.query(f'ab {current:.1f} {fi:.3f}')
        if not resp.startswith('A'):
            raise CommunicationError(resp)

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

fi_electrical = np.linspace(0, 2 * np.pi, 30)
fi_mechanical = []
current = 20

for fi_ele in fi_electrical:
    inv.set_ab(current, fi_ele)
    time.sleep(1)
    fi_mech = inv.res_angle()
    fi_mechanical.append(fi_mech)
    print(f'{fi_ele:.3f} -> {fi_mech:.3f}')

plt.plot(fi_electrical, fi_mechanical, 'o')
plt.show()