from usb_can_adapter_v1 import UsbCanAdapter
from enum import Enum, auto



def swap32(x):
    return int.from_bytes(x.to_bytes(4, byteorder='little'), byteorder='big', signed=False)

#port types will be u
class cdi_port_type(Enum):
    port_type_int_32 = 0
    port_type_float = 1


# port_types = [port_type_int_32,]

def get_value_and_type(channel,data):
    if channel.datatype == int:
        # return int(swap32(data.int()))
        return int.from_bytes(data, byteorder='little', signed=True)
    elif channel.datatype == float:
        return float(swap32(data))
    
    pass

class channel:
    def __init__(self,id,type):
        self.id = id
        self.datatype = type
        self.value = 0
        data = 0

class cdi_interface:
    def __init__(self,usb_can_adapter):
        self.channels = []
        if usb_can_adapter.device_port==None:
            return -1
        self.uca = usb_can_adapter
        pass
        # tty = 

    def parse_data(self):
        pass

    def add_channel(self, id, type, color):
        obj = channel(id, type)
        self.channels.append(obj)
        pass

    def update(self):
        received = self.uca.frame_receive()
        if received!=-1 :                                   #valid frame received
            data = self.uca.extract_data(self.uca.frame)  
            
            std_id = int(data['frame_id'][-4:],16) #get two LSB containing standard ID
            for channel in self.channels:
                if channel.id == std_id:
                    # print(self.uca.frame)  
                    # print(self.uca.frame[-(received-4):-1]) #Not playing with the library
                    channel.value = get_value_and_type(channel,self.uca.frame[-(received-4):-1])
                    pass
                    # print(channel.value)




