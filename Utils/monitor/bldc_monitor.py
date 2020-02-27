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

# class BLDCMonitor:

def signalSource():
    scopespd = Scope.scopes[0]; scopeact = Scope.scopes[1]
    spdAnno = scopespd.ax.annotate(
            '0.00', xy=(1, 0), va='center', ha="left",
            xycoords=('axes fraction',"data"),
            bbox=dict(boxstyle="round,pad=0.1", fc="#ADD6FF", ec="none", alpha=0.7))
    scopespd.ax.grid(True)
    scopeact.ax.set_ymargin(0.1)
    scopeact.canvasFlag = True
    with Receiver(info_pb2.Info, True) as receiver:
        while True:
            msg = receiver.nextMessage()
            scopespd.addData("y", msg.timestamp/100.0, msg.pid_info.y_real)
            scopeact.addData("x", msg.timestamp/100.0, msg.pid_info.x)
            spdAnno.set_y(msg.pid_info.y_real)
            spdAnno.set_text("{:.1f}".format(msg.pid_info.y_real))

def main():
    fig = plt.figure()
    fig.canvas.set_window_title("BLDC Monitor")
    axspd = plt.subplot2grid((2,1), (0,0))
    axact = plt.subplot2grid((2,1), (1,0), sharex=axspd)
    # fig, ax = plt.subplots(2,1)
    Scope(axspd, {"ylim":(-5,80)})
    Scope(axact, {"ylim":(-0.25,1.25)})

    anmation = animation.FuncAnimation(fig, Scope.updateAll, interval=100)

    thread = threading.Thread(target=signalSource)
    thread.setDaemon(True)
    thread.start()

    plt.show()


if __name__ == "__main__":
    main()
