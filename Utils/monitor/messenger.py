'''
messenger.py

Distributed under MIT License
Copyright (c) 2020 Haoze Zhang | Brown Engineering
'''
import serial
from serial import Serial
from serial.tools import list_ports
from google.protobuf.message import DecodeError

class Messenger(object):
    """ Delimitered serial protobuf message passing class. """
    config = {
        "preamble": b"UUUAT",
        "delimiter": b"\0\0\0AT",
        "baud": 1843200,
        "bytesize": serial.EIGHTBITS,
        "parity": serial.PARITY_ODD,
        "stopbits": serial.STOPBITS_TWO
    }

    def __init__(self, recvCls, useLast=False, config={}):
        self.device = self.__class__.selectSerial(useLast)
        self.ser = None
        self.recvCls = recvCls
        self.config = self.__class__.config
        for k,v in config.items():
            self.config[k] = v
        self.sendRaw = None
        self.sendMsg = None
        self.recvRaw = None
        self.recvMsg = None
        self.sendCount = 0
        self.sendErrCount = 0
        self.recvCount = 0
        self.recvErrCount = 0

    def __enter__(self):
        self.ser = Serial(
            self.device, self.config["baud"], self.config["bytesize"],
            self.config["parity"], self.config["stopbits"])
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        return self

    def __exit__(self, type, value, traceback):
        self.ser.close()
        self.ser = None

    def _parseMsg(self):
        container = self.recvCls()
        try:
            container.ParseFromString(self.raw)
            self.recvCount += 1
        except DecodeError:
            print("A message was not able to be decoded.")
            self.recvErrCount += 1
        return container

    def _readRawMsg(self):
        self.ser.read_until(self.config["preamble"])
        delimiter = self.config["delimiter"]
        return self.ser.read_until(delimiter)[:-len(delimiter)]

    def receive(self):
        self.raw = self._readRawMsg()
        self.msg = self._parseMsg()
        return self.msg

    def send(self, container):
        self.sendMsg = container
        self.sendRaw = self.sendMsg.SerializeToString()
        self.ser.write(self.config["preamble"])
        self.ser.write(self.sendRaw)
        self.ser.write(self.config["delimiter"])
        self.ser.flush()

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

    with Messenger(info_pb2.Info, True) as messenger:
        while True:
            messenger.receive()
            print(f"Raw data:\n {messenger.raw}")
            print(f"Message:\n {messenger.msg}")

if __name__ == "__main__":
    main()
