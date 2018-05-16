#!/usr/bin/env python3

import argparse
import serial
import sys
import json
import pickle

import threading
import datetime

import numpy as np

import time

from jy901 import JY901,AngleFrame,AccelFrame,DumbDevice

verbose_output = False

class DrawPQT:
    def __init__(self, dev, verbose=False):
        self.verbose_output = verbose
        self.dev = dev

    def update_scroll(self, acc):
        self.plots['norm'][1][:-1] = self.plots['norm'][1][1:]
        self.plots['norm'][1][-1] = np.linalg.norm(acc)
        self.plots['norm'][0].setData(self.plots['norm'][1])

        for (i,p) in enumerate(['x', 'y', 'z']):
            self.plots[p][1][:-1] = self.plots[p][1][1:]
            self.plots[p][1][-1] = acc[i]
            self.plots[p][0].setData(self.plots[p][1])

    def update(self):
        ## update 3D view
        angle = self.dev.next_angle()
        rot = angle.get_transform()

        cur_acc = self.dev.next_accel().accel
        new_acc = np.append([[0,0,0]], [cur_acc], axis=0)

        if self.verbose_output: print(cur_acc)
        if self.verbose_output: print(angle)

        self.accel.setData(pos=new_acc)
        self.accel.setTransform(rot)
        self.axis.setTransform(rot)
        # m1.setTransform(rot)        

        ## update scrolling data
        self.update_scroll(cur_acc)

    def buttonClicked(self, btn):
        if btn.isChecked():
            btn.setText("Recording")
            self.text.setReadOnly(True)
            self.dev.start_record(self.text.text())
        else:
            btn.setText("Start")
            self.text.setReadOnly(False)
            self.dev.stop_record()

    def drawpqt(self, dev):

        from pyqtgraph.Qt import QtCore, QtGui
        import pyqtgraph.opengl as gl
        import pyqtgraph as pg
        from pyqtgraph.dockarea import DockArea,Dock

        app = QtGui.QApplication([])

        ## window with scrolling data
        win_scroll = pg.GraphicsWindow()
        win_scroll.setWindowTitle('Scroll data')

        self.plots = {}
        for p in  ['x', 'y', 'z', 'norm']:
            plt = win_scroll.addPlot()
            plt.setRange(xRange=(0,300), yRange=(-6,6), disableAutoRange=True)
            pltdata = np.random.normal(size=300)

            self.plots[p] = (plt.plot(pltdata), pltdata)

        ### START 3D
        ## window with 3D view
        gl_view = gl.GLViewWidget()
        
        xgrid = gl.GLGridItem()
        ygrid = gl.GLGridItem()
        zgrid = gl.GLGridItem()
        gl_view.addItem(xgrid)
        gl_view.addItem(ygrid)
        gl_view.addItem(zgrid)

        ## rotate x and y grids to face the correct direction
        xgrid.rotate(90, 0, 1, 0)
        ygrid.rotate(90, 1, 0, 0)

        ## scale each grid differently
        xgrid.scale(0.2, 0.1, 0.1)
        ygrid.scale(0.2, 0.1, 0.1)
        zgrid.scale(0.1, 0.2, 0.1)

        init_accel = np.array([
            [0,0,0],
            [0,0,-1]
        ])

        self.accel = gl.GLLinePlotItem(pos=init_accel, width=3)
        self.axis = gl.GLAxisItem()

        # view.addItem(m1)
        gl_view.addItem(self.axis)
        gl_view.addItem(self.accel)
        ## END OF 3D

        #### START
        ## Define a top-level widget to hold everything
        w = QtGui.QWidget()

        ## Create some widgets to be placed inside
        btn = QtGui.QPushButton('Start')
        self.text = QtGui.QLineEdit('')
        btn.clicked.connect(lambda:self.buttonClicked(btn))
        btn.setCheckable(True)

        ## Create a grid layout to manage the widgets size and position
        layout = QtGui.QGridLayout()
        w.setLayout(layout)

        layout.addWidget(gl_view,    0, 0, 2, 3)
        layout.addWidget(win_scroll, 2, 0, 2, 3)
        layout.addWidget(btn,        4, 2, 1, 1)
        layout.addWidget(self.text,  4, 1, 1, 1)

        ## Display the widget as a new window
        w.show()
        #### END

        timer = QtCore.QTimer()
        timer.timeout.connect(self.update)
        timer.start(10)

        sys.exit(QtGui.QApplication.instance().exec_())

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

# class NumpyEncoder(json.JSONEncoder):
#     def default(self, obj):
#         if isinstance(obj, (list, dict, str, int, float, bool, type(None))):
#             return json.JSONEncoder.default(self, obj)
#         # if isinstance(obj, np.ndarray):
#         #     pass
#             # return obj.tolist()
#         return {'_python_object': pickle.dumps(obj)}

class ThreadWrap(threading.Thread):

    def __init__(self, device, exit_event):
        threading.Thread.__init__(self)
        self.dev = device
        self.last_frames = {}
        self.exit_event = exit_event
        self.dump_lock = threading.Lock()
        self.dump_file = None

    def run(self):
        while not exit_event.is_set():
            f = self.dev.next_frame()
            if self.dump_file:
                pickle.dump(f, self.dump_file)
                # self.dump_file.write(pickle.dumps(f))
            self.last_frames[f.__class__] = f

    def start_record(self, name):
        self.dump_lock.acquire()
        fname = "{}-{}".format(datetime.datetime.now().strftime('%G%m%d-%H_%M_%S'),name)
        self.dump_file = open(fname, 'wb')
        self.dump_lock.release()

    def stop_record(self):
        self.dump_lock.acquire()
        self.dump_file.close()
        self.dump_file = None
        self.dump_lock.release()

    def next_angle(self):
        return self.last_frames[AngleFrame]

    def next_accel(self):
        return self.last_frames[AccelFrame]


parser = argparse.ArgumentParser(description='CWT901CL utility.')

parser.add_argument('--dumb-device', action='store_true',
                    help='Do not use real data, use random.')

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

if args.dumb_device:
    jy901_dev = DumbDevice(10)
    orig_dev = jy901_dev
else:
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
    dpqt = DrawPQT(jy901_dev, verbose_output)
    dpqt.drawpqt(jy901_dev)
elif not args.use_threads:
    while True:
        f = jy901_dev.next_frame()
        print(f)
else:
    exit_event.set()
    jy901_dev.join()
    print("Doing nothing")
    
if not args.dumb_device:
    ser.close()


