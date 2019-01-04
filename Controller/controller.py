#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Dec 28 17:51:46 2018

@author: aahunter
"""

import math
import numpy as np
import serial
import io
import pylab
import threading
import time
from pysm import StateMachine, State, Event


class Robot(object):
    ORIENT_TIME = 10
    def __init__(self):
        dev = "/dev/cu.usbserial-DA01233R"
        baudRate = 56700
        self.mission_active = True
        self.orientation = math.radians(90)  #desired orientation is east
        self.sm = self._get_state_machine()
        self.timer1 = threading.Timer(Robot.ORIENT_TIME, self.on_timeout)
        self.ser = serial.Serial(dev,baudRate,timeout=1) #open serial port
        self.sio = io.TextIOWrapper(io.BufferedRWPair(self.ser, self.ser),encoding='ascii')
        self.ser.reset_input_buffer()
        self.X = np.zeros(3) # X = [x,y,theta]
        self.odo = np.zeros(7)
        self.getOdometry()
        heading = math.radians(360 - self.odo[3])
        heading = self.normalizeYaw(heading)
        self.X[2] = heading
        self.trajectory = self.X # store a record of states for plotting
        self.u = 100 #commanded velocity
        self.Kp = 1  #TODO calculate optimal Kp (yaw gain constant)

    def _get_state_machine(self):
        
        #define the states
        robot = StateMachine('Robot')
        init = State('Init')
        orient = State('Orient')
        error = State('Error')
        off = State('Off')
        
        #add states to state machine
        robot.add_state(init, initial = True)
        robot.add_state(orient)
        robot.add_state(error)
        robot.add_state(off)
        
        #internal transitions
        robot.add_transition(orient, None, events = 'odoRX',action = self.compareHeading)
        #m.add_transition(s0, None, events='i', action=action_i)
        
        #external transitions
        robot.add_transition(init, orient, events = ['odoRX'])
        robot.add_transition(orient,off, events=['oriented'])
        robot.add_transition(orient,error, events=['timeout'])
        
        #attach event handlers to each state
        orient.handlers = {'odoRX': self.compareHeading, 
                           'enter': self.on_orient_enter,
                           'exit':self.on_exit}
        off.handlers = {'enter': self.offState_on_enter,'exit': self.end_mission}
        
        # Attach enter/exit handlers
        states = [init,error]
        for state in states:
            state.handlers = {'enter': self.offState_on_enter, 'exit': self.on_exit}

        robot.initialize()
        return robot 
    
    #event handlers
    def on_timeout(self,state,event):
        print('Timeout...')
        self.sm.dispatch(Event('timeout', robot=self))
        
    def on_enter(self,state,event):
        print('entry to {0}'.format(state.name))
        
    def on_exit(self,state,event):
        print('exit from {0}'.format(state.name))
        
    def on_orient_enter(self,state,event):
        print('entry to {0}'.format(state.name))
        self.drive(-50,50)
        heading = math.radians(360 - self.odo[3])
        heading = self.normalizeYaw(heading)
        self.X[2] = heading
        
    def offState_on_enter(self,state,event):
        print('entry to {0}'.format(state.name))
        self.drive(0,0)
        
    def compareHeading(self,state,event):
        tol = math.radians(10)
        desired_heading = self.orientation
        current_heading = self.X[2]
        print('current_heading: {0}'.format(current_heading))
        print('desired_heading: {0}'.format(desired_heading))
        if (current_heading > desired_heading - tol) and (current_heading < desired_heading + tol):
            self.sm.dispatch(Event('oriented'))
            
    def end_mission(self,state,event):
        print('exit from {0}'.format(state.name))
        self.drive(0,0)
        self.mission_active = False
        
    #module functions    
    def drive(self,u_l,u_r):
        print('driving')
        v = (int(u_l),int(u_r))
        commandStr = '{0[0]:d},{0[1]:d}\n'.format(v)
        bytes_written = self.sio.write(commandStr)
        self.sio.flush()
        print(commandStr)
        print('bytes written: {0}'.format(bytes_written))
        #TODO add acknowledge
        
    def getOdometry(self):
        '''odometry string is [ dx dy dTheta Heading mag_x mag_y dTheta_gyro ] '''
        datasize = 7 #number of variables coming from robot
        s = self.sio.readline()
        #print (s)
        data = s.split(' ') 
        if (data[0] == '[') and (data[datasize+1] == ']\n') :
            for i in range(datasize):
                self.odo[i] = float(data[i+1])
            self.sm.dispatch(Event('odoRX',robot=self))
    
    def updateState(self):
        dx = self.odo[0]
        dy = self.odo[1]
        dTheta = self.odo[2]
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
            
def test_Robot():
    robot = Robot()
    while(robot.mission_active):
        robot.getOdometry()
        robot.updateState()
        #print(robot.X)
    return
        
if __name__ == '__main__':
    test_Robot()
            
            