#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jan  6 08:43:32 2019

@author: aahunter
"""

from transitions import Machine
import math
import numpy as np
import serial
import io
import time


dev = "/dev/cu.usbserial-DA01233R"
baudRate = 56700


class Robot(object):
    ODO_SIZE = 5
    TIMEOUT = 1
    LOOP_TIME = 0.1
    tol = math.radians(10) #orientation tolerance in degrees
    def __init__(self):
        self.mission_active = True
        self.desired_orientation = math.radians(90)  #desired orientation is west
        self.ser = serial.Serial(dev,baudRate,timeout=1) #open serial port
        self.sio = io.TextIOWrapper(io.BufferedRWPair(self.ser, self.ser),encoding='ascii')
        self.ser.reset_input_buffer()
        self.X = np.zeros(3) # X = [x,y,theta]
        self.trajectory = np.zeros(3) # record of states for plotting
        self.odo = np.zeros(self.ODO_SIZE)
        self.u = [0,0]
        self.Kp = 1  #TODO calculate optimal Kp (yaw  proportional gain)
        self.Ki = 10 #TODO tune optimal Ki (yaw integral gain)
        
    def getHeading(self):
        self.u = [0,0]
        self.drive()
        if(self.getOdo()):
            self.updateState()
            self.X[2] = math.radians(self.odo[3])
            print('heading: {}'.format(math.degrees(self.X[2])))
            self.to_orient()
            return 1
        return 0
    
    def on_enter_orient(self):  
        '''sets the robot spinning CCW after checking its orientation with the 
        desired setting'''
        #get current heading
        current_heading = math.radians(self.odo[3])
        #compare to desired orientation
        #if within the tolerance value issue 'oriented' event
        if(abs(self.desired_orientation - current_heading) < self.tol):
            self.oriented()
            return 1
       # else not oriented--set the robot spinning for another looptime      
        self.u = [-75,75]
        #start rotating CCW
        self.drive()
        #set current time
        current_time = time.time()
        if(self.getOdo()):
            self.updateState()
            print(self.X)
            self.odoRX()
            elapsed_time = time.time() - current_time
            remaining_time = self.LOOP_TIME - elapsed_time
            #sleep for remaining time in loop
            time.sleep(remaining_time)
            return 1
        return 0 

    def on_enter_end(self):
        #if the robot is moving, command it to stop and wait for a loop 
        self.u = [0,0]
        if (self.odo[0]):
            self.drive()
            #set current time
            current_time = time.time()
            if(self.getOdo()):
                self.updateState()
                print(self.X)
                self.odoRX()
                elapsed_time = time.time() - current_time
                remaining_time = self.LOOP_TIME - elapsed_time
                #sleep for remaining time in loop
                time.sleep(remaining_time)
                return 1
        print('Final robot pose: {}'.format(robot.X))
        return 1 
    
    def drive(self):
        '''sends controller command and also causes the robot to return
        odometry data which should be read immediately from getOdo'''
        commandStr = '[{0[0]:d},{0[1]:d}]'.format(self.u)
        self.ser.reset_output_buffer()
        bytes_written = self.sio.write(commandStr)
        self.sio.flush()
        print(commandStr)
        print('bytes written: {0}'.format(bytes_written))
        
    def getOdo(self):
        '''retrieves serial data from robot.  Will only return a value if a
        drive command is sent first'''
        s = self.readln()
        print (s)
        if s=='timeout':return 0
        data = s.split(' ') 
        if (data[0] == '[') and (data[self.ODO_SIZE+1] == ']') :
            for i in range(self.ODO_SIZE):
                self.odo[i] = float(data[i+1])
            return 1
    
    def updateState(self):
        dx = self.odo[0]
        dy = self.odo[1]
        dTheta = math.radians(self.odo[2])  #dTheta is in degrees from robot
        dX = np.array([dx,dy,dTheta])
        theta = self.X[2]
        c = math.cos(theta)
        s = math.sin(theta)
        R = np.array([[c,-s,0],[s,c,0],[0,0,1]]) #augmented rotation matrix
        self.X = R @ dX.T + self.X #rotate odometry to world coords and add to state
        self.X[2] = self.normalizeYaw(self.X[2])
        
    def normalizeYaw(self,yaw):
        '''keep yaw angle to +/- pi'''
        if yaw > math.pi:
            yaw -= 2*math.pi
        if yaw  < -math.pi:
            yaw += 2*math.pi
        return yaw
    
    def readln(self):
        '''reads serial port data from robot and returns it as a 
        string'''
        response = []
        char = self.ser.read()
#        print('char is {}'.format(char))
        while (char != b'\n'):
            if (char == b''): return 'timeout'
            response.append(char.decode())
            char = self.ser.read()
        response = ''.join(response)
        return response

robot = Robot()

states = ['start','orient','end','error']
# transitions take the form of ['trigger','source','dest'] and can be an 
# explicit dictionary or a list of lists
transitions = [
        ['odoRX', 'orient','orient'],
        ['oriented','orient','end'],
        ['odoRX','end','end']
        ]
#create the state machine
statemachine = Machine(robot, states=states, transitions=transitions, initial='start',queued=True)

#add event callbacks

robot.getHeading()
print(robot.state)


