#!/usr/bin/env python3

import argparse
import serial

import numpy as np

import time

from jy901 import JY901

def draw2d(dev, interval, window_width):
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    from matplotlib import style
    style.use('fivethirtyeight')


    fig = plt.figure()
    ax1 = fig.add_subplot(1,1,1)
    xdata, ydata = ([],[])
    start = time.time()

    def animate(i):
        global xdata,ydata
        af = dev.next_accel()

        y = af.get_norm()
        print(af)
        x = time.time() - start
        xdata.append(x)
        ydata.append(y)
        if len(xdata) == window_width :
            xdata = xdata[1:]
            ydata = ydata[1:]
        ax1.clear()
        ax1.plot(xdata, ydata)

    ani = animation.FuncAnimation(fig, animate, interval=interval)
    plt.show()


def draw3d(dev, interval, window_width):
    import matplotlib.pyplot as plt
    import mpl_toolkits.mplot3d.axes3d as p3
    import matplotlib.animation as animation

    # Attaching 3D axis to the figure
    fig = plt.figure()
    ax = p3.Axes3D(fig)

    # Setting the axes properties
    ax.set_xlim3d([0.0, 1.0])
    ax.set_xlabel('X')

    ax.set_ylim3d([0.0, 1.0])
    ax.set_ylabel('Y')

    ax.set_zlim3d([0.0, 1.0])
    ax.set_zlabel('Z')

    ax.set_title('Test')

    x = np.array([[0.0,1],
                  [0,0],
                  [0,0]])
    y = np.array([[0.0,0],
                  [0,1],
                  [0,0]])
    z = np.array([[0.0,0],
                  [0,0],
                  [0,1]])

    acc = np.array([[0.0, 0],
                    [0.0, 0],
                    [0.0, 0]])

    # display sensor referencial
    xp = ax.plot(x[0,0:2], x[1, 0:2], x[2, 0:2])[0]
    yp = ax.plot(y[0,0:2], y[1, 0:2], y[2, 0:2])[0]
    zp = ax.plot(z[0,0:2], z[1, 0:2], z[2, 0:2])[0]
    accp = ax.plot(acc[0,0:2], acc[1, 0:2], acc[2, 0:2])[0]

    def update_ref(num, ref, artists):
        rot = dev.next_angle().get_rotation_matrix()
        cur_acc = dev.next_accel().accel

        ref[3][:,1] = cur_acc

        for r,art in zip(ref, artists):
            r = rot @ r
            art.set_data(r[0:2, :2])
            art.set_3d_properties(r[2, :2])

        return artists

    # Creating the Animation object
    line_ani = animation.FuncAnimation(fig, update_ref, fargs=((x,y,z,acc),(xp,yp,zp,accp)), interval=interval)

    plt.show()


parser = argparse.ArgumentParser(description='CWT901CL utility.')

parser.add_argument('--device', type=str, default='/dev/rfcomm0',
                    help='tty device')

parser.add_argument('--rate', type=int, default=115200,
                    help='baud rate')

parser.add_argument('--draw', action='store_true',
                    help='enable graphical display')

parser.add_argument('--draw3d', action='store_true',
                    help='enable 3D graphical display')

parser.add_argument('--draw-interval', type=int, default=10,
                    help='interval between redraw of frame')

parser.add_argument('--draw-window', type=int, default=20,
                    help='width of window (in # of values)')


args = parser.parse_args()

ser = serial.Serial(args.device, args.rate)
jy901_dev = JY901(ser)

if args.draw:
    draw2d(jy901_dev, args.draw_interval, args.draw_window)
elif args.draw3d:
    draw3d(jy901_dev, args.draw_interval, args.draw_window)
else:
    while True:
        f = jy901_dev.next_frame()
        print(f)

ser.close()


