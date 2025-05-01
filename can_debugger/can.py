from enum import Enum
import serial
from dataclasses import dataclass
import struct

class CANUSB_SPEED(Enum):
    SPEED_1000000 = 0x01
    SPEED_800000 = 0x02
    SPEED_500000 = 0x03
    SPEED_400000 = 0x04
    SPEED_250000 = 0x05
    SPEED_200000 = 0x06
    SPEED_125000 = 0x07
    SPEED_100000 = 0x08
    SPEED_50000 = 0x09
    SPEED_20000 = 0x0a
    SPEED_10000 = 0x0b
    SPEED_5000 = 0x0c


class CANUSB_MODE(Enum):
    NORMAL = 0x00
    LOOPBACK = 0x01
    SILENT = 0x02
    LOOPBACK_SILENT = 0x03


class CANUSB_FRAME(Enum):
    STANDARD = 0x01
    EXTENDED = 0x02


class CANUSB_PAYLOAD_MODE(Enum):
    INJECT_PAYLOAD_MODE_RANDOM = 0
    INJECT_PAYLOAD_MODE_INCREMENTAL = 1
    INJECT_PAYLOAD_MODE_FIXED = 2

class SerialException(Exception):
    pass


@dataclass
class CANFrame:
    id: int
    data: bytearray
    

class CANUSB:
    def __init__(self, port: str, speed: CANUSB_SPEED):
        self.serial_device = serial.Serial(port, baudrate=2_000_000, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_TWO, timeout=None)
        self.__configure_can_interface(speed=speed)

    def get_frame(self):
        received = self.__receive_frame()
        data = self.__parse_frame(received)  
        return data
    
    def __parse_frame(self, frame: bytearray) -> CANFrame:
        frame_id, = struct.unpack("<H", frame[2:4])
        data = frame[-(len(frame)-4):-1]

        return CANFrame(frame_id, data)

    def __configure_can_interface(self, speed, mode: CANUSB_MODE = CANUSB_MODE.NORMAL):

            """
            Configures the CAN-to-serial adapter settings.

            Parameters:
            speed (CANUSB_SPEED, optional): The speed of the CAN bus in bps.
            mode (CANUSB_MODE, optional): The mode in which the CAN to serial adapter operates.
            frame (CANUSB_FRAME, optional): The frame format.

            Fixed 20-byte Communication Protocol:
            - Packet Header: 0xaa, 0x55
            - Type: 0x01 for Data
            - Frame Type: 0x01 for Standard frame, 0x02 for Extended frame.
            - Frame Format: 0x01 for Data frame, 0x02 for Remote frame.
            - Frame ID Data: 4 bytes, high bytes at the front, low bytes at the back.
            - Frame Data Length: The data length of the CAN bus that is sent or accepted.
            - Frame Data: Up to 8 data bytes.
            - Reserve: 0x00
            - Check Code: Checksum from frame type to error code, accumulating and taking the low 8 bits.
            """

            cmd_frame = bytearray()

            cmd_frame.append(0xaa)
            cmd_frame.append(0x55)
            cmd_frame.append(0x12)

            cmd_frame.append(speed.value)
            cmd_frame.append(CANUSB_FRAME.STANDARD.value)
            cmd_frame.extend([0] * 8)  # Fill with zeros for Filter ID and Mask ID (not handled)
            cmd_frame.append(mode.value)
            cmd_frame.extend([0x01, 0, 0, 0, 0])
            cmd_frame.append(self.__generate_checksum(cmd_frame[2:19]))

            self.__send_frame(cmd_frame)
    
    @staticmethod
    def __generate_checksum(data: bytearray) -> int:
        """Generates a checksum from the given data bytearray."""
        checksum = sum(data)
        return checksum & 0xff
    
    def __send_frame(self, frame: bytearray):
        """Sends a frame to the USB-CAN-ADAPTER device."""
        if not self.serial_device.is_open:
            raise SerialException("Serial port is not open.")
        try:
            self.serial_device.write(bytes(frame))
        except serial.SerialException as e:
            raise SerialException(f"Serial write() failed: {e}")
    
    def __receive_frame(self) -> int:
        """Receives a frame from the USB-CAN-ADAPTER device."""
        if not self.serial_device.is_open:
            raise SerialException("Serial port is not open.")

        frame = bytearray()
        started = False

        while len(frame) < 20:
            try:
                byte = self.serial_device.read(1)[0]
            except serial.SerialException as e:
                raise SerialException(f"Serial write() failed: {e}")

            if byte == 0xaa:
                started = True

            if started:
                frame.append(byte)

            if byte == 0x55 and started:
                break

        return frame