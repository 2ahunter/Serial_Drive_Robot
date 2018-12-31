#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Dec 28 17:51:46 2018

@author: aahunter
"""

import math
import keyboard
import numpy as np
import serial
import io
import pylab
import threading
import time
from pysm import StateMachine, State, Event


class Robot(object):

    def __init__(self):
        dev = "/dev/cu.usbserial-DA01233R"
        baudRate = 56700
        self.ser = serial.Serial(dev,baudRate,timeout=1) #open serial port
        self.sio = io.TextIOWrapper(io.BufferedRWPair(self.ser, self.ser))
        self.ser.reset_input_buffer()
        self.state = np.zeros(3) # state = [x,y,theta]
        self.odo = np.zeros(7)
        self.getOdometry()
        heading = math.radians(360 - self.odo[3])
        self.state[2] = heading
        self.normalizeYaw()
        self.trajectory = self.state # store a record of states for plotting
        self.u = 100 #commanded velocity
        self.Kp = 1  #TODO calculate optimal Kp (yaw gain constant)

        
    def getOdometry(self):
        '''odometry string is [ dx dy dTheta Heading mag_x mag_y dTheta_gyro ] '''
        datasize = 7 #number of variables coming from robot
        s = self.sio.readline()
        print (s)
        data = s.split(' ') 
        if (data[0] == '[') and (data[datasize+1] == ']\n') :
            for i in range(datasize):
                self.odo[i] = float(data[i+1])
    
    def updateState(self):
        dx = self.odo[0]
        dy = self.odo[1]
        dTheta = self.odo[2]
        dX = np.array([dx,dy,dTheta])
        theta = self.state[2]
        c = math.cos(theta)
        s = math.sin(theta)
        R = np.array([[c,-s,0],[s,c,0],[0,0,1]]) #augmented rotation matrix
        self.state = R @ dX.T + self.state #rotate odometry to world coords and add to state
        self.normalizeYaw()
        
    def normalizeYaw(self):
        '''keep yaw angle to +/- pi'''
        if self.state[2] > math.pi:
            self.state[2] = self.state[2] -math.pi
        if self.state[2] < -math.pi:
            self.state[2] = self.state[2] +math.pi
            
def test_Robot():
    robot = Robot()
    while(True):
        robot.getOdometry()
        robot.updateState()
        print (robot.state)
        
        
if __name__ == '__main__':
    test_Robot()
            
            