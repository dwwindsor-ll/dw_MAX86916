#!/usr/bin/env python

#from https://thepoorengineer.com/en/arduino-python-plot/

from threading import Thread
import serial
import time
import collections
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import struct
import copy
import pandas as pd
import numpy as np


class serialPlot:
    def __init__(self, serialPort='/dev/ttyUSB0', serialBaud=115200, plotLength=100):
        self.port = serialPort
        self.baud = serialBaud
        self.plotMaxLength = plotLength
        self.rawData = ""
        self.dataType = None
        self.data = []
        self.isRun = True
        self.isReceiving = False
        self.thread = None
        self.plotTimer = 0
        self.previousTimer = 0
        # self.csvData = []

        print('Trying to connect to: ' + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
        try:
            self.serialConnection = serial.Serial(serialPort, serialBaud, timeout=4)
            print('Connected to ' + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
            self.numPlots=len(self.serialConnection.readline().decode("utf-8").split())
            print(f"Number of plots is: {self.numPlots}")
            for i in range(self.numPlots):   # give an array for each type of data and store them in a list
                self.data.append(collections.deque([0] * plotLength, maxlen=plotLength))
        except:
            print("Failed to connect with " + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')

    def readSerialStart(self):
        if self.thread == None:
            self.thread = Thread(target=self.backgroundThread)
            self.thread.start()
            # Block till we start receiving values
            while self.isReceiving != True:
                time.sleep(0.1)

    def getSerialData(self, frame, lines, lineValueText, pltNumber, axes):
        if pltNumber == 0:  # in order to make all the clocks show the same reading
            currentTimer = time.perf_counter()
            self.plotTimer = int((currentTimer - self.previousTimer) * 1000)     # the first reading will be erroneous
            self.previousTimer = currentTimer
        self.privateData = copy.deepcopy(self.rawData)    # so that the 3 values in our plots will be synchronized to the same sample time
        data = self.privateData.decode("utf-8").split() 
        value = data[pltNumber].split(':')
        self.data[pltNumber].append(int(value[1]))    # we get the latest data point and append it to our array
        lines.set_data(range(self.plotMaxLength), self.data[pltNumber])
        lineValueText.set_text('[' + value[0] + '] = ' + value[1])
        axes.relim()
        axes.autoscale()

    def backgroundThread(self):    # retrieve data
        time.sleep(1.0)  # give some buffer time for retrieving data
        self.serialConnection.reset_input_buffer()
        while (self.isRun):
            self.rawData=self.serialConnection.readline()
            self.isReceiving = True
            # print(self.rawData)

    def close(self):
        self.isRun = False
        self.thread.join()
        self.serialConnection.close()
        print('Disconnected...')
        # df = pd.DataFrame(self.csvData)
        # df.to_csv('/home/rikisenia/Desktop/data.csv')

def makeFigure(xLimit, yLimit, subplot):
    xmin, xmax = xLimit
    ymin, ymax = yLimit
    fig = plt.figure()
    axs = [None] * subplot
    for i in range(subplot):
        if subplot == 1:
            axs[i] = fig.add_subplot(1, 1, 1+i) 
        elif subplot == 2:
            axs[i] = fig.add_subplot(2, 1, 1+i) 
        elif 2 < subplot <= 4:
            axs[i] = fig.add_subplot(2, 2, 1+i) 
        elif 4 < subplot <= 6:
            axs[i] = fig.add_subplot(3, 2, 1+i) 
        elif 6 < subplot <= 9:
            axs[i] = fig.add_subplot(3, 3, 1+i)
        elif 9 < subplot <= 12:
            axs[i] = fig.add_subplot(3, 4, 1+i) 
        else:
            print("Failed to make enough subplots")
        axs[i].set_axis_off()   
        axs[i].set_xlim(xmin, xmax)
        axs[i].set_ylim(ymin, ymax)
        axs[i].margins(0.05)
    return fig, axs

def main():
    portName = 'COM5'
    baudRate = 115200 
    maxPlotLength = 100     # number of points in x-axis of real time plot
    s = serialPlot(portName, baudRate, maxPlotLength)   # initializes all required variables
    s.readSerialStart()                                               # starts background thread

    # plotting starts below
    pltInterval = 50    # Period at which the plot animation updates [ms]
    xLimit = (0, maxPlotLength)
    yLimit = (0, 100000)
    style = ['m-', 'r-', 'g-','b-', 'k-', 'm-', 'r-', 'g-','b-']  # linestyles for the different plots
    while len(style) < s.numPlots:
        style.append('k-')
    anim = []
    fig, axs = makeFigure(xLimit, yLimit, s.numPlots)
    for i in range(s.numPlots):
        lines = axs[i].plot([], [], style[i])[0]
        lineValueText = axs[i].text(0.50, 0.90, '', transform=axs[i].transAxes)
        anim.append(animation.FuncAnimation(fig, s.getSerialData, fargs=(lines, lineValueText, i, axs[i]), interval=pltInterval, blit=False), )  # fargs has to be a tuple
    plt.show()

    s.close()


if __name__ == '__main__':
    main()