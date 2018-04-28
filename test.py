#!/usr/bin/env python3

import argparse
import serial

import threading

import numpy as np

import time

from jy901 import JY901,AngleFrame,AccelFrame

verbose_output = False

def drawpqt(dev):
    global verbose_output

    from pyqtgraph.Qt import QtCore, QtGui
    import pyqtgraph.opengl as gl
    import pyqtgraph as pg

    app = QtGui.QApplication([])

    view = gl.GLViewWidget()
    view.show()
    
    xgrid = gl.GLGridItem()
    ygrid = gl.GLGridItem()
    zgrid = gl.GLGridItem()
    view.addItem(xgrid)
    view.addItem(ygrid)
    view.addItem(zgrid)

    ## rotate x and y grids to face the correct direction
    xgrid.rotate(90, 0, 1, 0)
    ygrid.rotate(90, 1, 0, 0)

    ## scale each grid differently
    xgrid.scale(0.2, 0.1, 0.1)
    ygrid.scale(0.2, 0.1, 0.1)
    zgrid.scale(0.1, 0.2, 0.1)

    ## Example 1:
    ## Array of vertex positions and array of vertex indexes defining faces
    ## Colors are specified per-face

    # verts = np.array([
    #     [0, 0, 0],
    #     [2, 0, 0],
    #     [1, 2, 0],
    #     [1, 1, 1],
    # ])
    # faces = np.array([
    #     [0, 1, 2],
    #     [0, 1, 3],
    #     [0, 2, 3],
    #     [1, 2, 3]
    # ])
    # colors = np.array([
    #     [1, 0, 0, 0.3],
    #     [0, 1, 0, 0.3],
    #     [0, 0, 1, 0.3],
    #     [1, 1, 0, 0.3]
    # ])

    # ## Mesh item will automatically compute face normals.
    # m1 = gl.GLMeshItem(vertexes=verts, faces=faces, faceColors=colors, smooth=False)
    # m1.translate(2, 2, 2)
    # m1.setGLOptions('additive')

    init_accel = np.array([
        [0,0,0],
        [0,0,-1]
    ])
    accel = gl.GLLinePlotItem(pos=init_accel, width=3)
    axis = gl.GLAxisItem()

    # view.addItem(m1)
    view.addItem(axis)
    view.addItem(accel)

    def update():
        angle = dev.next_angle()
        rot = angle.get_transform()

        cur_acc = dev.next_accel().accel

        new_acc = np.append([[0,0,0]], [cur_acc], axis=0)

        if verbose_output: print(cur_acc)
        if verbose_output: print(angle)

        accel.setData(pos=new_acc)
        accel.setTransform(rot)
        axis.setTransform(rot)
        # m1.setTransform(rot)        

    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(10)
    
    QtGui.QApplication.instance().exec_()

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

class ThreadWrap(threading.Thread):

    def __init__(self, device, exit_event):
        threading.Thread.__init__(self)
        self.dev = device
        self.last_frames = {}
        self.exit_event = exit_event

    def run(self):
        while not exit_event.is_set():
            f = self.dev.next_frame()
            self.last_frames[f.__class__] = f

    def next_angle(self):
        return self.last_frames[AngleFrame]

    def next_accel(self):
        return self.last_frames[AccelFrame]


parser = argparse.ArgumentParser(description='CWT901CL utility.')

parser.add_argument('--device', type=str, default='/dev/rfcomm0',
                    help='tty device')

parser.add_argument('--verbose', action='store_true',
                    help='Enable verbose output.')

parser.add_argument('--use-threads', action='store_true',
                    help='Use separate thread for reading sensor')

parser.add_argument('--rate', type=int, default=115200,
                    help='baud rate')

parser.add_argument('--pqg-draw', action='store_true',
                    help='enable graphical display with pygtgraph')

parser.add_argument('--mpl-draw', action='store_true',
                    help='enable matplotlib graphical display')

parser.add_argument('--mpl-draw3d', action='store_true',
                    help='enable matplotlib 3D graphical display')

parser.add_argument('--mpl-draw-interval', type=int, default=10,
                    help='interval between redraw of matplotlib frame')

parser.add_argument('--mpl-draw-window', type=int, default=20,
                    help='width of matplotlib window (in # of values)')

args = parser.parse_args()

ser = serial.Serial(args.device, args.rate)

jy901_dev = JY901(ser)
orig_dev = jy901_dev

if args.verbose:
    verbose_output = True

if args.use_threads:
    exit_event = threading.Event()
    jy901_dev = ThreadWrap(orig_dev, exit_event)
    jy901_dev.start()

if args.mpl_draw:
    draw2d(jy901_dev, args.mpl_draw_interval, args.mpl_draw_window)
elif args.mpl_draw3d:
    draw3d(jy901_dev, args.mpl_draw_interval, args.mpl_draw_window)
elif args.pqg_draw:
    drawpqt(jy901_dev)
elif not args.use_threads:
    while True:
        f = jy901_dev.next_frame()
        print(f)
else:
    exit_event.set()
    jy901_dev.join()
    print("Doing nothing")
    
ser.close()


