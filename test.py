#!/usr/bin/env python3

import math
import serial

import numpy.linalg

import argparse

parser = argparse.ArgumentParser(description='CWT901CL utility.')

parser.add_argument('--device', type=str, default='/dev/rfcomm0',
                    help='tty device')

parser.add_argument('--rate', type=int, default=115200,
                    help='baud rate')

args = parser.parse_args()

ser = serial.Serial(args.device, args.rate)

BAD_FRAME_COUNT=0

def read_frame():
    global BAD_FRAME_COUNT
    while True:
        frame=bytearray()

        while True:
            byte = ser.read()[0]
            if byte == 0x55:
                break

        frame.append(byte)

        for i in range(10):
            b = ser.read()[0]
            frame.append(b)

        s = sum(frame[0:10]) % 256

        if s == frame[10]:
            return frame
        else:
            BAD_FRAME_COUNT = BAD_FRAME_COUNT + 1

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

while True:
    f = read_frame()
    if f[1] == 0x51:
        print(f)
        af = AccelFrame(f)
        print("")
        print(af)

ser.close()


