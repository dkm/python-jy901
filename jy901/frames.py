import math
import numpy.linalg

import numpy as np
import math

import pyqtgraph as pg

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
        """Returns a new AngleFrame object from the bytearray `byts`
        Angles are in degrees.
        """

        self.roll = (int.from_bytes(byts[2:4], byteorder='little', signed=True) / 32768) * 180
        self.pitch = (int.from_bytes(byts[4:6], byteorder='little', signed=True) / 32768) * 180
        self.yaw = (int.from_bytes(byts[6:8], byteorder='little', signed=True) / 32768) * 180

        self.temp = int.from_bytes(byts[8:10], byteorder='little', signed=True) /100

        self.frame = byts

    def __str__(self):
        return "angle=(roll:{:10.4f}, pitch:{:10.4f}, yaw:{:10.4f}), temp={:10.4f}".format(self.roll, self.pitch, self.yaw, self.temp)

    def get_transform(self):
         tr = pg.SRTTransform3D()
         tr.rotate(self.yaw , (0, 0, 1))
         tr.rotate(self.pitch, (0, 1, 0))
         tr.rotate(self.roll, (1, 0, 0))
         return tr

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

