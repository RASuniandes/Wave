# import serial

# import sys
# sys.path.append("/Library/Frameworks/Python.framework/Versions/3.11/lib/python3.11/site-packages")
import serial

import serial.tools.list_ports


class SerialObj:

    def __init__(self, serial_speed):
        self.serial = None
        self.baud_rate = serial_speed
        

    @staticmethod
    def get_ports():
        return list(serial.tools.list_ports.comports())

    def connect(self, port):
        self.serial = serial.Serial(port, self.baud_rate, timeout=1)
        self.serial.flushInput()

    def is_connect(self):
        return self.serial.isOpen()

    def get_data(self):
        if not self.serial.isOpen():
            return None
        else:
            return self.serial.readline()

    def disconnect(self):
        if self.serial is None:
            return

        self.serial.close()
        