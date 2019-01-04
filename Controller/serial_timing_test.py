#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jan  2 15:14:34 2019

@author: aahunter
"""
import serial
import io
import time
import random
import numpy as np

dev = "/dev/cu.usbserial-DA01233R"
baudRate = 56700
ser = serial.Serial(dev,baudRate,timeout=.10) #open serial port
sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser))

tests = 100
times = np.zeros(tests)
start = 0.0
stop = 0.0
a = -255
b = 255
    

def drive(u_l,u_r):
    v = (int(u_l),int(u_r))
    commandStr = '[{0[0]:d},{0[1]:d}]'.format(v)
    ser.reset_output_buffer()
    sio.write(commandStr)
    sio.flush()
    s = sio.readline()
    print (s[:-1])
    
def getOdometry():
    '''odometry string is [ dx dy dTheta Heading mag_x mag_y dTheta_gyro ] '''
    commandStr = 'o'
    ser.reset_output_buffer()
    sio.write(commandStr)
    sio.flush()
    s = sio.readline()
    print (s[:-1])


for i in range(tests):
#    v_left = random.randint(a, b)
#    v_right = random.randint(a, b)
    v_left = -100
    v_right = 100
    start = time.monotonic()
#    getOdometry()
    drive(v_left,v_right)
    stop = time.monotonic()
    times[i] = stop - start


drive(0,0)
ser.close()
print('min response time: {0}, max time: {1}, mean time: {2}'.format(min(times),max(times),np.average(times)))
    
