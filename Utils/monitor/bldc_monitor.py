'''
bldc_monitor.py

Distributed under MIT License
Copyright (c) 2020 Haoze Zhang | Brown Engineering
'''

import os
import time
import threading

import yaml
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from scope import Scope
from messenger import Messenger

import cmd_pb2
import info_pb2

cmdFilePath = "cmd.yaml"

def populateCmd(cmdData, mid):
    cmd = cmd_pb2.Cmd()
    cmd.id = mid
    pid_cmd = cmd.pid_cmd
    for k,v in cmdData["pid_cmd"].items():
        setattr(pid_cmd, k, v)
    return cmd

def loadCmdFile(messenger):
    lastModTime = os.path.getmtime(cmdFilePath)
    mid = 0
    while True:
        if os.path.getmtime(cmdFilePath) > lastModTime:
            with open(cmdFilePath, "r") as f:
                cmd = yaml.load(f, Loader=yaml.Loader)
                cmdContainer = populateCmd(cmd, mid)
                messenger.send(cmdContainer)
                print(f"Command sent:\n{cmdContainer}")
            lastModTime = os.path.getmtime(cmdFilePath)
            mid += 1
        time.sleep(0.5)


def signalSource(messenger):
    scopespd = Scope.scopes[0]; scopeact = Scope.scopes[1]
    spdAnno = scopespd.ax.annotate(
            '0.00', xy=(1, 0), va='center', ha="left",
            xycoords=('axes fraction',"data"),
            bbox=dict(boxstyle="round,pad=0.1", fc="#ADD6FF", ec="none", alpha=0.7))
    scopespd.ax.grid(True)
    scopeact.ax.set_ymargin(0.1)
    scopeact.canvasFlag = True
    while True:
        msg = messenger.receive()
        scopespd.addData("y", msg.timestamp/100.0, msg.pid_info.y_real)
        scopeact.addData("x", msg.timestamp/100.0, msg.pid_info.x)
        spdAnno.set_y(msg.pid_info.y_real)
        spdAnno.set_text("{:.1f}".format(msg.pid_info.y_real))

def main():
    fig = plt.figure()
    fig.canvas.set_window_title("BLDC Monitor")
    axspd = plt.subplot2grid((2,1), (0,0))
    axact = plt.subplot2grid((2,1), (1,0), sharex=axspd)
    Scope(axspd, {"ylim":(-5,80)})
    Scope(axact)

    aniation = animation.FuncAnimation(fig, Scope.updateAll, interval=100)

    with Messenger(info_pb2.Info, True) as messenger:
        thread = threading.Thread(target=signalSource, args=(messenger,))
        thread.setDaemon(True)
        thread.start()

        thread = threading.Thread(target=loadCmdFile, args=(messenger,))
        thread.setDaemon(True)
        thread.start()

        plt.show()


if __name__ == "__main__":
    main()
