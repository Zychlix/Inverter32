from usb_can_adapter_v1 import UsbCanAdapter

import can_debug_interface



if __name__ == "__main__":
    
    uca = UsbCanAdapter()
    # uca.command_settings(CANUSB_FRA)
    uca.set_port('/dev/ttyUSB0')
    uca.adapter_init()
    uca.command_settings(speed=500000)

    
    cdi = can_debug_interface.cdi_interface(uca)


    cdi.add_channel(0x100,int,0)

    print("Python CAN Debugger v0.1 \r\n")
    while True:
        #frame_len = uca.frame_receive()

        cdi.update()

        # print("Frame: \r\n")
        # print('length %d \r\n',frame_len)
        # if frame_len!=-1 : #Ok frame
        #     data = uca.extract_data(uca.frame)
        #     print('frame id: %d \r\n', data['frame_id'])
        #     print(data['data']);

        