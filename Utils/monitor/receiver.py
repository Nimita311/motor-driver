'''
receiver.py

Distributed under MIT License
Copyright (c) 2020 Haoze Zhang | Brown Engineering
'''
from serial import Serial
from serial.tools import list_ports
from google.protobuf.message import DecodeError

class Receiver(object):
    """ Delimitered serial protobuf message receiver/decoder. """
    config = {
        "preamble": b"UUUAT",
        "delimiter": b"\0\0\0AT",
        "baud": 115200
    }

    def __init__(self, msgClass, useLast=False, config=None):
        self.device = self.__class__.selectSerial(useLast)
        self.ser = None
        self.msgClass = msgClass
        self.config = config if config is not None else self.__class__.config
        self.raw = None
        self.msg = None
        self.count = 0
        self.errCount = 0

    def __enter__(self):
        self.ser = Serial(self.device, self.config["baud"])
        return self

    def __exit__(self, type, value, traceback):
        self.ser.close()
        self.ser = None

    def nextMessage(self):
        self.raw = self._readRawMsg()
        self.msg = self._parseMsg()
        return self.msg

    def _parseMsg(self):
        container = self.msgClass()
        try:
            container.ParseFromString(self.raw)
            self.count += 1
        except DecodeError:
            print("A message was not able to be decoded.")
            self.errCount += 1
        return container

    def _readRawMsg(self):
        self.ser.flush()
        self.ser.read_until(self.config["preamble"])
        delimeter = self.config["delimiter"]
        return self.ser.read_until(delimeter)[:-len(delimeter)]

    @staticmethod
    def selectSerial(useLast=False):
        ports = list_ports.comports()
        if useLast:
            return ports[-1].device
        print("Please select a serial ports by ID:")
        for i, port in enumerate(ports):
            print(f"{i:2d}: {port.device}")
        while True:
            try:
                portIdx = int(input("ID to connect: "))
                if portIdx < len(ports):
                    break
                else:
                    print("This ID does not exist.")
            except ValueError:
                print("Please input an ID number.")
        selectedDevice = ports[portIdx].device
        print(f"Selected port: {selectedDevice}")
        return selectedDevice

def main():
    ''' A demo that prints raw data and the decoded message to stdout. '''
    import info_pb2

    with Receiver(info_pb2.Info, True) as receiver:
        while True:
            receiver.nextMessage()
            print(f"Raw data:\n {receiver.raw}")
            print(f"Message:\n {receiver.msg}")

if __name__ == "__main__":
    main()
