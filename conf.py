#!/usr/bin/env python3

import argparse
import serial
import sys
import time

from jy901.device import JY901

parser = argparse.ArgumentParser(description='CWT901CL utility.')

parser.add_argument('--dumb-device', action='store_true',
                    help='Do not use real data, use random.')

parser.add_argument('--device', type=str, default='/dev/rfcomm0',
                    help='tty device')

parser.add_argument('--frames', type=str, default='time,acceleration,angular,angle',
                    help='frames types to request from sensor.')

parser.add_argument('--verbose', action='store_true',
                    help='Enable verbose output.')

parser.add_argument('--rate', type=int, default=115200,
                    help='baud rate')

parser.add_argument('--return-rate', type=int,
                    help='return rate in Hz')

parser.add_argument('--gyro-calibrate', type=int,
                    help='enter gyro/accel calibration mode, use value as timeout')

parser.add_argument('--reset-default', action='store_true',
                    help='resets all config to defaults.')

args = parser.parse_args()

ser = serial.Serial(args.device, args.rate)
jy901_dev = JY901(ser)

if args.reset_default:
    jy901_dev.save(True)
    sys.exit(0)

if args.return_rate:
    jy901_dev.set_return_rate(args.return_rate)
    print ("Set return rate to {}".format(args.return_rate))
    jy901_dev.save()
    sys.exit(0)

if args.gyro_calibrate:
    jy901_dev.set_calibration('gyro_acc')
    print("Start calibration")
    for i in range(args.gyro_calibrate*10):
        f = jy901_dev.next_frame()
        print(f)
        time.sleep(0.1)
    print("End calibration")
    jy901_dev.set_calibration('exit')
        



