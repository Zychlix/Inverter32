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
        self.ser = serial.Serial(port, baud, timeout=2)

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

unwrapping_offset = 0

N_POINTS = 48

fi_electrical = np.concatenate((np.linspace(0, 2 * np.pi, N_POINTS),np.linspace(2 * np.pi,0, N_POINTS) ))
fi_mechanical = []
current = 40

for fi_ele in fi_electrical:
    inv.set_ab(current, fi_ele)
    time.sleep(1)
    fi_mech_orig = inv.res_angle()
    fi_mech = fi_mech_orig + unwrapping_offset
    if fi_mechanical and np.abs(fi_mech - fi_mechanical[-1]) > np.pi:
        unwrapping_offset += np.pi*2*np.sign(fi_mechanical[-1] - fi_mech )
        fi_mech = fi_mech_orig + unwrapping_offset
    # if fi_mech < 0:
    #     fi_mech += 2*np.pi
    # if fi_mech > 2*np.pi:
    #     fi_mech -= 2*np.pi
    fi_mechanical.append(fi_mech)
    print(f'{fi_ele:.3f} -> {fi_mech:.3f}')

b, a = np.polyfit(fi_electrical, fi_mechanical, deg=1)
print(a, b)



inv.set_ab(0, 0)

plt.plot(fi_electrical, a + b * fi_electrical, color="k", lw=2.5)
plt.plot(fi_electrical, fi_mechanical, 'o')
plt.show()