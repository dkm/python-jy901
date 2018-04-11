#!/usr/bin/env python3

import argparse
import serial

from jy901 import JY901

parser = argparse.ArgumentParser(description='CWT901CL utility.')

parser.add_argument('--device', type=str, default='/dev/rfcomm0',
                    help='tty device')

parser.add_argument('--rate', type=int, default=115200,
                    help='baud rate')

args = parser.parse_args()


ser = serial.Serial(args.device, args.rate)

jy901_dev = JY901(ser)

while True:
    af = jy901_dev.next_accel()
    print(af)

ser.close()


