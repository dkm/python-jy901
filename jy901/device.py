import random
import time

from .frames import *

class DumbDevice:
    """
    Same interface, simply returns random data.
    """

    def __init__(self, freq):

        self.next_frame_idx = 0
        self.period = 1 / freq

        self.cur_angle = (0,0,0)
        self.cur_temp = 25
        self.cur_acc = (0,0,-1)

    def next_frame(self):
        self.next_frame_idx = (self.next_frame_idx + 1) % 2
        time.sleep(self.period)

        if self.next_frame_idx == 0:
            return self.next_accel()
        else:
            return self.next_angle()

    def next_accel(self):
        frame = bytearray()
        frame.append(0x55)
        frame.append(0x51)
        ## Ax
        frame += int((self.cur_acc[0]/16)*32768).to_bytes(2, byteorder='little', signed=True)
        ## Ay
        frame += int((self.cur_acc[1]/16)*32768).to_bytes(2, byteorder='little', signed=True)
        ## Az
        frame += int((self.cur_acc[2]/16)*32768).to_bytes(2, byteorder='little', signed=True)
        ## Temp
        frame += self.cur_temp.to_bytes(2, byteorder='little', signed=True)

        self.cur_acc = list(map(lambda x: (x + (random.random()/10-0.05))%4, self.cur_acc))
        return AccelFrame(frame)

    def next_angle(self):
        frame = bytearray()
        frame.append(0x55)
        frame.append(0x53)
        ## roll / 180 * 32768
        frame += int(((self.cur_angle[0]/180)*32768)).to_bytes(2, byteorder='little', signed=True)
        ## pitch
        frame += int(((self.cur_angle[1]/180)*32768)).to_bytes(2, byteorder='little', signed=True)
        ## yaw
        frame += int(((self.cur_angle[2]/180)*32768)).to_bytes(2, byteorder='little', signed=True)
        ## Temp
        frame += self.cur_temp.to_bytes(2, byteorder='little', signed=True)

        self.cur_angle = list(map(lambda x: (x+(random.random()-0.5)) , self.cur_angle))
        return AngleFrame(frame)

class JY901:
    """
    Driver for JY901 device.
    """

    def __init__(self, source):
        """Returns a new JY901 device object. `source` must be read/write object."""

        self.source = source
        self.bad_frame = 0

    def read_frame(self):
        """Returns the next valid frame as a bytearray read from the source."""

        while True:
            frame=bytearray()

            while True:
                byte = self.source.read()[0]
                if byte == 0x55:
                    break

            frame.append(byte)

            for i in range(10):
                b = self.source.read()[0]
                frame.append(b)

            s = sum(frame[0:10]) % 256

            if s == frame[10]:
                return frame
            else:
                self.bad_frame = self.bad_frame + 1

    def next_frame(self):
        """Returns the next frame as a *Frame object."""
        while True:
            f = self.read_frame()
            if f[1] in FRAME_MAP:
                return FRAME_MAP[f[1]](f)

    def next_accel(self):
        """Returns the next AccelFrame read from the source."""

        while True:
            f = self.read_frame()
            if f[1] == 0x51:
                return AccelFrame(f)

    def next_angle(self):
        """Returns the next AngleFrame read from the source."""

        while True:
            f = self.read_frame()
            if f[1] == 0x53:
                return AngleFrame(f)

    def set_baud_rate(self, baud):
        """Sets the baudrate of the underlying JY901 device."""

        bs = bytearray()
        bs.append(0xFF)
        bs.append(0xAA)
        bs.append(0x04)

        convert = {
            2400: 0x00,
            4800: 0x01,
            9600: 0x02,
            19200: 0x03,
            38400: 0x04,
            57600: 0x05,
            115200: 0x06,
            230400: 0x07,
            460800: 0x08,
            921600: 0x09,
        }
        bs.append(convert[baud])
        bs.append(0)

        self.source.write(bs)

    def set_return_rate(self, hertz):
        """Sets the return rate of the underlying JY901 device."""

        bs = bytearray()
        bs.append(0xFF)
        bs.append(0xAA)
        bs.append(0x03)

        convert = {
            0.1: 0x01,
            0.5: 0x02,
            1  : 0x03,
            2  : 0x04,
            5  : 0x05,
            10 : 0x06,
            20 : 0x07,
            50 : 0x08,
            100: 0x09,
            200: 0x0a,
            999: 0x0b,
            0  : 0x0c
        }
        bs.append(convert[hertz])
        bs.append(0)
        self.source.write(bs)

    def set_return_content(self, enabled):
        """Sets the return content of the underlying device.
        `enabled` must be a set containing the selected features from:
        'time', 'acceleration', 'angular', 'angle', 'magnetic', 'atmo', 'latlon', 'gps', 'port'.
        Beware that not all features are always available.
        """

        bs = bytearray()
        bs.append(0xFF)
        bs.append(0xAA)
        bs.append(0x02)
        v = 0

        if 'time' in enabled:
            v = v | 1<<0
        if 'acceleration' in enabled:
            v = v | 1<<1
        if 'angular' in enabled:
            v = v | 1<<2
        if 'angle' in enabled:
            v = v | 1<<3
        if 'magnetic' in enabled:
            v = v | 1<<4
        if 'atmo' in enabled:
            v = v | 1<<4
        if 'latlon' in enabled:
            v = v | 1<<5
        if 'gps' in enabled:
            v = v | 1<<6
        if 'port' in enabled:
            v = v | 1<<7

        bs.append(v)
        bs.append(0x00)
        self.source.write(bs)
