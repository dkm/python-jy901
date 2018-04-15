import math
import numpy.linalg

import numpy as np
import math

class AccelFrame:
    """
    Describes an Acceleration frame.

    `accel` field contains a 3D `np.array` with acceleration values.
    `temp` field contains the temperature
    """

    def __init__(self, byts):
        """Returns a new AccelFrame object from the bytearray `byts`"""

        a = np.array([int.from_bytes(byts[2:4], byteorder='little', signed=True),
                      int.from_bytes(byts[4:6], byteorder='little', signed=True),
                      int.from_bytes(byts[6:8], byteorder='little', signed=True)])

        self.accel = (a/32768)*16

        self.temp = int.from_bytes(byts[8:10], byteorder='little', signed=True) /100

        self.frame = byts

    def get_norm(self):
        """Returns the acceleration norm."""

        return numpy.linalg.norm(self.accel)

    def __str__(self):
        n = self.get_norm()
        return "acc=({0:10.4f}, {1:10.4f}, {2:10.4f})~{3:10.4f}, temp={4:10.4f}".format(self.accel[0], self.accel[1], self.accel[2], n, self.temp)


class AngleFrame:
    """
    Describes an Angle frame.

    `roll`, `pitch` and  `yaw` fields contains the respective angles.
    `temp` contains the temperature
    """

    def __init__(self, byts):
        """Returns a new AngleFrame object from the bytearray `byts`"""

        self.roll = int.from_bytes(byts[2:4], byteorder='little', signed=True) / 32768
        self.pitch = int.from_bytes(byts[4:6], byteorder='little', signed=True) / 32768
        self.yaw = int.from_bytes(byts[6:8], byteorder='little', signed=True) / 32768

        self.temp = int.from_bytes(byts[8:10], byteorder='little', signed=True) /100

        self.frame = byts

    def __str__(self):
        return "angle=(roll:{:10.4f}, pitch:{:10.4f}, yaw:{:10.4f}), temp={:10.4f}".format(self.roll, self.pitch, self.yaw, self.temp)

    def get_rotation_matrix(self):
        """Returns the corresponding rotation matrix Z-Y-X."""

        yawMatrix = np.array([
            [math.cos(self.yaw), -math.sin(self.yaw), 0],
            [math.sin(self.yaw), math.cos(self.yaw), 0],
            [0, 0, 1]
        ])

        pitchMatrix = np.array([
            [math.cos(self.pitch), 0, math.sin(self.pitch)],
            [0, 1, 0],
            [-math.sin(self.pitch), 0, math.cos(self.pitch)]
        ])

        rollMatrix = np.array([
            [1, 0, 0],
            [0, math.cos(self.roll), -math.sin(self.roll)],
            [0, math.sin(self.roll), math.cos(self.roll)]
        ])
        return yawMatrix @ pitchMatrix @ rollMatrix

class MagneticFrame:
    """
    Describes a Magnetic frame.

    `h` contains a 3D `np.array` with magnetic field values.
    `temp` contains the temperature.
    """

    def __init__(self, byts):
        """Returns a new MagneticFrame from bytearray `byts`"""

        self.h = [int.from_bytes(byts[2:4], byteorder='little', signed=True),
                  int.from_bytes(byts[4:6], byteorder='little', signed=True),
                  int.from_bytes(byts[6:8], byteorder='little', signed=True)]

        self.temp = int.from_bytes(byts[8:10], byteorder='little', signed=True) /100

        self.frame = byts

    def __str__(self):
        return "magnetic H=({:10.4f}, {:10.4f}, {:10.4f}), temp={:10.4f}".format(self.h[0], self.h[1], self.h[2], self.temp)

class AngularVelocityFrame:
    """
    Describes the Angular velocity frame.

    `w` contains a 3D `np.array` with the angular velocity values.
    `temp` contains the temperature
    """

    def __init__(self, byts):
        """Returns a new AngularVelocityFrame from bytearray `byts`"""
        w = [int.from_bytes(byts[2:4], byteorder='little', signed=True),
             int.from_bytes(byts[4:6], byteorder='little', signed=True),
             int.from_bytes(byts[6:8], byteorder='little', signed=True)]
        self.w = list(map(lambda x: (x/32768), w))

        self.temp = int.from_bytes(byts[8:10], byteorder='little', signed=True) /100

        self.frame = byts

    def __str__(self):
        return "angular velocity=(wx:{:10.4f}, wy:{:10.4f}, wz:{:10.4f}), temp={:10.4f}".format(self.w[0], self.w[1], self.w[2], self.temp)

class AtmosphericFrame:
    """
    Describes the Atmospheric frame.

    `pa` contains the atmospheric pressure in Pascal (Pa).
    `height` contains the height in meters (m).
    """

    def __init__(self, byts):
        """Returns a new AtmosphericFrame object from bytearray `byts`"""

        self.pa = int.from_bytes(byts[2:6], byteorder='little', signed=True)
        self.height = int.from_bytes(byts[6:10], byteorder='little', signed=True)
        self.frame = byts

    def __str__(self):
        return "Pa = {:10.4f}, Height = {:10.4f}".format(self.pa, self.height)

FRAME_MAP = {
    0x51 : AccelFrame,
    0x52 : AngularVelocityFrame,
    0x53 : AngleFrame,
    0x54 : MagneticFrame,
    0x56 : AtmosphericFrame,
}

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
