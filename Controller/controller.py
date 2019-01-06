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
    ORIENT_TIME = 1
    LOOP_TIME = 0.1
    ODO_SIZE = 5
    def __init__(self):
        dev = "/dev/cu.usbserial-DA01233R"
        baudRate = 56700
        TIMEOUT = 0.1
        self.mission_active = True
        self.orientation = math.radians(90)  #desired orientation is west
        self.sm = self._get_state_machine()
        self.timer1 = threading.Timer(self.ORIENT_TIME, self.on_timeout)
        self.timer1.start()
#        self.timer2 = threading.Timer(self.LOOP_TIME, self.Post_event('loop_timeout'))
#        self.timer2.start()
        self.ser = serial.Serial(dev,baudRate,timeout=TIMEOUT) #open serial port
        self.sio = io.TextIOWrapper(io.BufferedRWPair(self.ser, self.ser),encoding='ascii')
        self.ser.reset_input_buffer()
        self.X = np.zeros(3) # X = [x,y,theta]
        self.trajectory = np.zeros(3) # record of states for plotting
        self.odo = np.zeros(self.ODO_SIZE)
        self.u = [0,0]
        self.Kp = 1  #TODO calculate optimal Kp (yaw  proportional gain)
        self.Ki = 10 #TODO tune optimal Ki (yaw integral gain)

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
        
        
        #external transitions
        robot.add_transition(init, orient, events = ['loop_timeout'],action=self.loop_timer)
        robot.add_transition(init,error, events=['timeout'])
        robot.add_transition(orient,off, events=['oriented'])
        robot.add_transition(orient,error, events=['timeout'])
        
        #attach event handlers to each state
        init.handlers = {'enter': self.on_enter,
                         'exit': self.on_exit}
        
        orient.handlers = {'enter': self.on_orient_enter,
                           'loop_timeout': self.compareHeading,
                           'exit': self.on_exit}
        
        off.handlers = {'enter': self.on_enter,
                        'loop_timeout': self.Off_wait,
                        'exit': self.on_exit}
        
        error.handlers = {'enter': self.on_enter,
                         'exit': self.on_exit}
        
#        # define the events for event checker TODO: do I need this?
#        robot.events.append('odoRx')
#        robot.events.append('oriented')
#        robot.events.append('loop_timeout')
#        robot.events.append('timeout')
        
        robot.initialize()
        return robot 
    
    #event handlers 
    def on_enter(self,state,event):
        print('entry to {0}'.format(state.name))
    
    def on_event(self,state,event):
        print('in Init, event: {}'.format(event))
        
    def on_exit(self,state,event):
        print('exit from {0}'.format(state.name))
        
    def on_orient_enter(self,state,event):
        print('entry to {0}'.format(state.name))
        heading = math.radians(360 - self.odo[3])
        heading = self.normalizeYaw(heading)
        self.X[2] = heading
        
    def Off_wait(self,state,event):
        print('entry to {0}'.format(state.name))
        print(self.X)
        if(self.odo[0]):
            self.loop_timer()
        else:
            print('Robot stopped')
            self.mission_active == False
        
    def compareHeading(self,state,event):
        tol = math.radians(10)
        desired_heading = self.orientation
        current_heading = self.X[2]
        print('current_heading: {0}'.format(current_heading))
        print('desired_heading: {0}'.format(desired_heading))
        if (current_heading > desired_heading - tol) and (current_heading < desired_heading + tol):
            self.Post_event('oriented')
            self.loop_timer(self,state,event)
            self.drive([0,0])
        else:
            self.loop_timer(self,state,event)
            self.drive([-50,50])

        
    #module functions    
    #TODO: change definition of drive to reflect bi directional communication
    # as it sends the command and receives the data
    
    def on_timeout(self):
#        if self.timer2.is_alive(): self.timer2.cancel()
        print('Timeout...')
        self.sm.dispatch(Event('loop_timeout', robot=self))
#        self.mission_active == False
        
    def loop_timer(self,state,event):
#        if self.timer2.is_alive(): self.timer2.cancel()
        timer2 = threading.Timer(self.LOOP_TIME, self.Post_event('loop_timeout'))
        timer2.start()

    
    def drive(self,u):
        commandStr = '[{0[0]:d},{0[1]:d}]'.format(u)
        self.ser.reset_output_buffer()
        bytes_written = self.sio.write(commandStr)
        self.sio.flush()
        print(commandStr)
        print('bytes written: {0}'.format(bytes_written))
        s = self.readln()
        print (s)
        data = s.split(' ') 
        if (data[0] == '[') and (data[self.ODO_SIZE+1] == ']') :
            for i in range(self.ODO_SIZE):
                self.odo[i] = float(data[i+1])
            self.updateState()
        return 0
        

# old version. keep for now as legacy     
#    def drive(self,u_l,u_r):
#        print('driving')
#        v = (int(u_l),int(u_r))
#        commandStr = '{0[0]:d},{0[1]:d}\n'.format(v)
#        bytes_written = self.sio.write(commandStr)
#        self.sio.flush()
#        print(commandStr)
#        print('bytes written: {0}'.format(bytes_written))
        
#    def getOdometry(self):
#        '''odometry string is [ dx dy dTheta Heading mag_x mag_y dTheta_gyro ] '''
#        datasize = 7 #number of variables coming from robot
#        s = self.sio.readline()
#        #print (s)
#        data = s.split(' ') 
#        if (data[0] == '[') and (data[datasize+1] == ']\n') :
#            for i in range(datasize):
#                self.odo[i] = float(data[i+1])
#            self.sm.dispatch(Event('odoRX',robot=self))
    
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
        print('New heading {}'.format(self.X[2]))
#        self.trajectory = np.stack((self.trajectory,self.X))
        
    def normalizeYaw(self,yaw):
        '''keep yaw angle to +/- pi'''
        if yaw > math.pi:
            yaw -= 2*math.pi
        if yaw  < -math.pi:
            yaw += 2*math.pi
        return yaw
    
    def readln(self):
        response = []
        char = self.ser.read()
        while (char != b'\n'):
            response.append(char.decode())
            char = self.ser.read()
        response = ''.join(response)
        return response

    def Post_event(self,eventStr):
        print('Event: {}'.format(eventStr))
        self.sm.dispatch(Event(eventStr, robot=self))
        return 1

            
def test_Robot():
    robot = Robot()
    while(robot.mission_active):
        pass
    return robot.trajectory
        
if __name__ == '__main__':
    test_Robot()
            
            