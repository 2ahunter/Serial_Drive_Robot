#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jan  2 15:14:34 2019

@author: aahunter
"""
import serial
import io


dev = "/dev/cu.usbserial-DA01233R"
baudRate = 56700
ser = serial.Serial(dev,baudRate,timeout=1) #open serial port
sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser),encoding='ascii')

def drive(u_l,u_r):
    v = (int(u_l),int(u_r))
    commandStr = '[{0[0]:d},{0[1]:d}]\n'.format(v)
#        commandStr = '-50,50 \n'
#        command = bytes(commandStr, 'ascii')
 #   sio.flush()
 #   ser.reset_output_buffer()
    sio.write(commandStr)
    sio.flush()
    sio.write(commandStr)
    sio.flush()

