import math
import numpy.linalg

class JY901:
    def __init__(self, source):
        self.source = source
        self.bad_frame = 0

    def read_frame(self):
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

    def next_accel(self):
        while True:
            f = self.read_frame()
            if f[1] == 0x51:
                return AccelFrame(f)

    def set_baud_rate(self, baud):
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

class AccelFrame:
    def __init__(self, byts):
        a = [
            int.from_bytes(byts[2:4], byteorder='little', signed=True),
            int.from_bytes(byts[4:6], byteorder='little', signed=True),
            int.from_bytes(byts[6:8], byteorder='little', signed=True)]
        self.accel = list(map(lambda x: (x/32768)*16, a))
        self.temp = int.from_bytes(byts[8:10], byteorder='little', signed=True) /100

        self.frame = byts

    def get_norm(self):
        return numpy.linalg.norm(self.accel)

    def __str__(self):
        n = self.get_norm()
        return "acc=({0:10.4f}, {1:10.4f}, {2:10.4f})~{3:10.4f}, temp={4:10.4f}".format(self.accel[0], self.accel[1], self.accel[2], n, self.temp)
