'''
scope.py

Distributed under MIT License
Copyright (c) 2020 Haoze Zhang | Brown Engineering
'''

class Scope(object):
    ''' Animated Matplotlib plot for live data '''
    scopes = list()
    config = {
        "xlim_width": 5,
        "ylim": [0, 100]
    }

    def __init__(self, ax, config=None):
        self.ax = ax
        self.config = config if config is not None else self.__class__.config
        self.lines = dict()
        self.data = dict()
        self.updatedNames = list()
        self.canvasFlag = True
        self.xlim = [0, self.config["xlim_width"]]
        self.ylim = self.config["ylim"]
        Scope.scopes.append(self)

    def addLine(self, name, xs=None, ys=None):
        xs = list() if xs is None else xs
        ys = list() if ys is None else ys
        newLine = self.ax.plot(xs, ys)[0]
        self.data[name] = (xs, ys)
        self.lines[name] = newLine
        self.updatedNames.append(name)

    def addData(self, name, x, y):
        if name in self.lines:
            xs, ys = self.data[name]
            xs.append(x); ys.append(y)
            self.updatedNames.append(name)
        else:
            self.addLine(name, [x], [y])

        if x > self.xlim[1]:
            width = self.xlim[1]-self.xlim[0]
            self.xlim[1] = x + 0.75*width
            self.xlim[0] = self.xlim[1] - width
            self.canvasFlag = True

    def update(self, frame):
        if self.canvasFlag:
            self.ax.set_xlim(self.xlim)
            self.ax.set_ylim(self.ylim)
            self.ax.figure.canvas.draw()
            self.canvasFlag = False

        for name in self.updatedNames:
            xs, ys = self.data[name]
            self.lines[name].set_data(xs, ys)

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
