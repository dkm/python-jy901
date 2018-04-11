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

class AccelFrame:
    def __init__(self, byts):
        a = [
            int.from_bytes(byts[2:4], byteorder='little', signed=True),
            int.from_bytes(byts[4:6], byteorder='little', signed=True),
            int.from_bytes(byts[6:8], byteorder='little', signed=True)]
        self.accel = list(map(lambda x: (x/32768)*16, a))
        self.temp = int.from_bytes(byts[8:10], byteorder='little', signed=True) /100

        self.frame = byts

    def __str__(self):
        n = numpy.linalg.norm(self.accel)
        return "acc=({0})~{1}, temp={2}".format(self.accel, n, self.temp)
