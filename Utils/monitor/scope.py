'''
scope.py

Distributed under MIT License
Copyright (c) 2020 Haoze Zhang | Brown Engineering
'''

from collections import deque

class Scope(object):
    ''' Animated Matplotlib plot for live data '''
    scopes = list()
    config = {
        "xlim_width": 5,
        "ts": 0.05,
        "xlim_step": 0.75,
        "ylim": None,
        "ymargin": 0.1
    }

    def __init__(self, ax, config={}):
        self.ax = ax
        self.config = dict(self.__class__.config)
        for k, v in config.items():
            self.config[k] = v
        self.datalen = int(self.config["xlim_width"]/self.config["ts"]*1.5)
        self.lines = dict()
        self.data = dict()
        self.updatedNames = list()
        self.xlim = [0, self.config["xlim_width"]]
        if self.config["ylim"] is None:
            self.ax.set_ymargin(self.config["ymargin"])
        else:
            self.ax.set_ylim(self.config["ylim"])
        self.canvasFlag = True
        Scope.scopes.append(self)

    def addLine(self, name):
        xs = deque(maxlen=self.datalen)
        ys = deque(maxlen=self.datalen)
        newLine = self.ax.plot(xs, ys)[0]
        self.data[name] = (xs, ys)
        self.lines[name] = newLine
        self.updatedNames.append(name)

    def addData(self, name, x, y):
        # Create a new line if the name does not exist
        if name not in self.lines:
            self.addLine(name)

        # Append new data point
        xs, ys = self.data[name]
        # Time revert, clear line and adjust xlim
        if xs and x < xs[-1]:
            xs.clear(); ys.clear()
            self.xlim = [x, x+self.config["xlim_width"]]
            self.canvasFlag = True
        xs.append(x); ys.append(y)
        self.updatedNames.append(name)

        # Beyond xlim, shift right
        if x > self.xlim[1]:
            width = self.config["xlim_width"]
            self.xlim[1] = x + self.config["xlim_step"]*width
            self.xlim[0] = self.xlim[1] - width
            self.canvasFlag = True

    def update(self, frame):
        if self.config["ylim"] is None:
            self.ax.relim()
            self.ax.autoscale_view()
            self.canvasFlag = True

        if self.canvasFlag:
            self.ax.set_xlim(self.xlim)
            self.ax.figure.canvas.draw()
            self.canvasFlag = False

        for name in self.updatedNames:
            self.lines[name].set_data(*self.data[name])

    @classmethod
    def updateAll(cls, frame):
        for scope in cls.scopes:
            scope.update(frame)

def main():
    ''' A demo that plots two iid random processes. '''
    import time;
    import random
    import threading
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation

    fig, ax = plt.subplots()
    scope = Scope(ax)
    anmation = animation.FuncAnimation(fig, scope.update, interval=50)

    def signalSource(scope):
        startTime = time.time()
        while True:
            x = time.time()-startTime
            y1 = random.uniform(0.0, 100.0)
            y2 = random.uniform(0.0, 100.0)
            scope.addData("Random1", x, y1)
            scope.addData("Random2", x, y2)
            time.sleep(0.05)
    thread = threading.Thread(target=signalSource, args=(scope,))
    thread.setDaemon(True)
    thread.start()
    plt.show()

if __name__ == "__main__":
    main()
