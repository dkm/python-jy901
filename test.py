#!/usr/bin/env python3

import argparse
import serial

import numpy as np
import time

from jy901 import JY901

parser = argparse.ArgumentParser(description='CWT901CL utility.')

parser.add_argument('--device', type=str, default='/dev/rfcomm0',
                    help='tty device')

parser.add_argument('--rate', type=int, default=115200,
                    help='baud rate')

parser.add_argument('--draw', action='store_true',
                    help='enable graphical display')

parser.add_argument('--draw-interval', type=int, default=10,
                    help='interval between redraw of frame')

parser.add_argument('--draw-window', type=int, default=20,
                    help='width of window (in # of values)')


args = parser.parse_args()

if args.draw:
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    from matplotlib import style
    style.use('fivethirtyeight')

ser = serial.Serial(args.device, args.rate)

jy901_dev = JY901(ser)

start = time.time()

if args.draw:
    fig = plt.figure()
    ax1 = fig.add_subplot(1,1,1)
    xdata, ydata = ([],[])
    def animate(i):
        global xdata,ydata
        af = jy901_dev.next_accel()
        y = af.get_norm()
        print(af)
        x = time.time() - start
        xdata.append(x)
        ydata.append(y)
        if len(xdata) == args.draw_window :
            xdata = xdata[1:]
            ydata = ydata[1:]
        ax1.clear()
        ax1.plot(xdata, ydata)
    ani = animation.FuncAnimation(fig, animate, interval=args.draw_interval)
    plt.show()

else:
    while True:
        af = jy901_dev.next_accel()
        y = af.get_norm()
        x = time.time() - start

        # print("{0},{1}".format(x,y))
        print(af)

ser.close()


