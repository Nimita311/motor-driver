'''
bldc_monitor.py

Distributed under MIT License
Copyright (c) 2020 Haoze Zhang | Brown Engineering
'''

import threading

import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation

import info_pb2
from scope import Scope
from receiver import Receiver

def signalSource(scope, receiver):
    while True:
        msg = receiver.nextMessage()
        scope.addData("y", msg.timestamp/100.0, msg.pid_info.y_real)

def main():
    fig, ax = plt.subplots()
    scope = Scope(ax)
    anmation = animation.FuncAnimation(fig, Scope.updateAll, interval=50)

    with serial.Serial(Receiver.selectSerial(), 115200) as ser:
        receiver = Receiver(ser, info_pb2.Info)
        thread = threading.Thread(target=signalSource, args=(scope, receiver))
        thread.setDaemon(True)
        thread.start()

    plt.show()

if __name__ == "__main__":
    main()
