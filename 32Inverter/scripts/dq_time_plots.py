import time

import serial
import numpy as np
import matplotlib.pyplot as plt
from enum import Enum
import os

class EvaluationType(Enum):
    D_ANALYSIS = 1
    Q_ANALYSIS = 2

class Measurement:
    def __init__(self):
        self.component = EvaluationType.Q_ANALYSIS
        self.voltage:float = 0
        self.frequency:float = 0
        self.data = []
        pass

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
    def arm(self, ):
        resp = self.query(f'arm')
        if not resp.startswith('OK'):
            raise CommunicationError(resp)
        
    def set_plot_channel(self, channel:int):
        resp = self.query(f'channel  {channel:.1f}')
        if not resp.startswith('OK'):
            raise CommunicationError(resp)
        
    def read_osc_data(self, points):
        resp = self.query(f'plot')
        a = 'a'
        while a:
            if(a.startswith('O') ):
                return
            try:
                a,b = (self.ser.readline().decode('utf-8').strip()).split(';')
            except:
                print('End of data stream')
                return
            points.append((a,b))
           

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
    def get_series(self, series:Measurement):
        self.set_plot_channel(0)
        if(series.component == EvaluationType.D_ANALYSIS):
            inv.set_vf(series.frequency, series.voltage,0)
        if(series.component  == EvaluationType.Q_ANALYSIS):
            inv.set_vf(series.frequency, 0, series.voltage)
        time.sleep(1)
        self.arm()
        time.sleep(1)
        inv.set_vf(series.frequency, 0, 0)
        data = []
        self.read_osc_data(series.data)
        return data
    
    def init_folder(self):
        path = '/home/zychlix/Desktop/silnik/pomiar_'
        dir_count = 0;
        new_path = path+str(dir_count)
        print(new_path)

        Exist = os.path.exists(new_path) 
        while(Exist):
            dir_count= dir_count +1
            new_path = path+str(dir_count)
            Exist = os.path.exists(new_path) 
        #clean folder

        self.path = new_path
        try:
            os.mkdir(new_path)
            print(f"Directory '{new_path}' created successfully.")
        except FileExistsError:
            print(f"Directory '{new_path}' already exists.")
        except PermissionError:
            print(f"Permission denied: Unable to create '{new_path}'.")
        except Exception as e:
            print(f"An error occurred: {e}")
        


    def save_csv(self,series:Measurement):
        name =f'pomiar_{series.frequency:.3f}Hz_{series.voltage:.3f}V'
        if(type == EvaluationType.Q_ANALYSIS):
            name+='Q'
        if(type == EvaluationType.D_ANALYSIS):
            name+='D'
        
        file = open(self.path+'/'+name, "w")

        file.write(f'Test: {series.voltage:.3f}V {series.frequency:.3f}Hz {series.component} Date: {time.ctime()}\r')


        for i, data in enumerate(series.data):
            file.write(f'{str(i)};{data[0]};{data[1]}\r')

        file.close()
        
        pass





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

inv.init_folder()

unwrapping_offset = 0

N_POINTS = 5
FREQ_LOW = 10
FREQ_HIGH = 1000
freq = np.linspace(10, 1000, N_POINTS)
fi_mechanical = []

current = 5

inv.set_plot_channel(0) #Y channel

#zrob gdzies klase pomiaru, zamiast przekazwyac wszystkie dane z reki jak malpa

series = Measurement()

series.component = EvaluationType.Q_ANALYSIS
series.voltage = 5;
series.frequency = 100;





for freq_ele in freq:
    series.frequency = freq_ele
    points = inv.get_series(series)
    inv.save_csv(series)    
    print(f'Series f={freq_ele}Hz saved \r')
    



b, a = np.polyfit(fi_electrical, fi_mechanical, deg=1)
print(a, b)



inv.set_ab(0, 0)

plt.plot(fi_electrical, a + b * fi_electrical, color="k", lw=2.5)
plt.plot(fi_electrical, fi_mechanical, 'o')
plt.show()