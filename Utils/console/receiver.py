'''
receiver.py

Distributed under MIT License
Copyright (c) 2020 Haoze Zhang | Brown Engineering
'''

from serial.tools import list_ports

class Receiver(object):
    """ Delimitered serial protobuf message receiver/decoder. """
    config = {
        "preamble": b"UUUAT",
        "delimiter": b"\0\0\0AT"
    }

    def __init__(self, ser, msgClass, config=None):
        self.ser = ser
        self.msgClass = msgClass
        self.config = config if config is not None else self.__class__.config
        self.raw = None
        self.msg = None
        self.count = 0
        self.errCount = 0

    def nextMessage(self):
        self.raw = self._readRawMsg()
        self.msg = self._parseMsg()
        return self.msg

    def _parseMsg(self):
        container = self.msgClass()
        try:
            container.ParseFromString(self.raw)
            self.count += 1
        except:
            self.errCount += 1
        return container

    def _readRawMsg(self):
        self.ser.flush()
        self.ser.read_until(self.config["preamble"])
        delimeter = self.config["delimiter"]
        return self.ser.read_until(delimeter)[:-len(delimeter)]

    @staticmethod
    def selectSerial():
        ports = list_ports.comports()
        print("Please select a serial ports by ID:")
        for i, port in enumerate(ports):
            print(f"{i:2d}: {port.device}")
        while True:
            try:
                portIdx = int(input("ID to connect: "))
                if portIdx < len(ports):
                    break
            except:
                pass
        selectedDevice = ports[portIdx].device
        print(f"Connecting to {selectedDevice}")
        return selectedDevice

def main():
    ''' A demo that prints raw data and the decoded message to stdout. '''
    import serial
    import info_pb2

    with serial.Serial(Receiver.selectSerial(), 115200) as ser:
        receiver = Receiver(ser, info_pb2.Info)
        while True:
            receiver.nextMessage()
            print(f"Raw data:\n {receiver.raw}")
            print(f"Message:\n {receiver.msg}")

if __name__ == "__main__":
    main()
