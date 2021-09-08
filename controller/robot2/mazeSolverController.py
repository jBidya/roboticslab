""" 
Final Project
Roll:204101072
Email: jbidyanath@iitg.ac.in
Problem Description : Maze solver robot swarm (in webots)
Algorithm Used: wall-following, Dead-End Avoider, Cross-Road Explorer
--------------------------CONTROLLER PROGRAM FOR ROBOT-(1)----------------
"""
# import sensor,motor,actuator,robots
import math
from controller import Robot,  DistanceSensor, Motor
import socket,json
import threading 
import time
import struct
from controller import Emitter
import pygeohash as gh 
import os
from pathlib import Path
import matplotlib.pyplot as plt

# Enable GPS

# Sample robot's location

TIME_STEP = 64
MAX_SPEED=6.28
# create the Robot instance.
robot = Robot()
# create a emitter at channel-line 1
emitter=robot.getEmitter("emitter")
emitter.setChannel(1)
# create a receiver at channel-line 2
receiver=robot.getReceiver("receiver")
# receiver side buffer stores the data sent over for a period of TIME_STEP 
receiver.enable(TIME_STEP) 
# Listen to emitter at channel-2
receiver.setChannel(2)

# get distancesensor values into list "ps"
ps = []
psNames = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]
for i in range(8):
    ps.append(robot.getDistanceSensor(psNames[i]))
    ps[i].enable(TIME_STEP)

#enable left and right motors and initialize position,velocity    
leftMotor = robot.getMotor('left wheel motor')
rightMotor = robot.getMotor('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)
# Enable GPS
gps = robot.getGPS('gps')
gps.enable(TIME_STEP)

# Store robot's location
xs = []
ys = []

# error terms for PID-controller
eprev = 0
ecurr = 0

#function to detect if cliff/obstacle is detected ahead or not?
"""
When cliff is ahead of the robot, this function return True otherwise False
The distance sensor ps0, ps7 and ps6 is used to identify if cliff is ahead?
"""
def cliff_detected(): 
    if ps[0].getValue() >100 or ps[7].getValue() >100:
        return True
    return False

# function to silently execute the action with pre-installed speed for "sec"-time unit    
"""
This method is called just to move along with inertia for the 'sec' time unit.
Robot moves with previous initialized speed. 
"""
def passive_wait(sec):
    inTime=robot.getTime()
    #print(sec,inTime)
    sec+=inTime
    while sec>inTime:
        inTime+=0.05
        robot.step(5)
# This function finds if there is wall to right of the robot or not
def walltoRight():
    w_d2=0.7
    ecurr = (w_d2*ps[2].getValue() + (1-w_d2)*ps[1].getValue())- 80
    if ecurr<-15:
        return True
# Robot STOPS at the location
def stop():
    print("STOP")
    leftSpeed = 0
    rightSpeed =0
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
    robot.step(TIME_STEP)  
# Robot take a turn of 90-degree CCW    
def turn90():
    stop()
    print("Turning 90-degree ccw:")
    leftSpeed = MAX_SPEED
    rightSpeed =-MAX_SPEED
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
# Robot is taking a turn of 270-degree CCW
def turn270():
    stop()
    print("Turning 270-degree ccw:")
    leftSpeed = -MAX_SPEED
    rightSpeed =MAX_SPEED
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
# Move a little forward
def step_little():
    leftSpeed = MAX_SPEED
    rightSpeed =MAX_SPEED
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
# Robot solves the maze using this function
def solve_maze():    
    global ecurr,eprev
    forward=0
    flag=0
    count=0                                                
    while robot.step(TIME_STEP) != -1:
        
        #send the current location of robot-2 to supervisor
        
        # Get the current location of robot in a Grid of size 12*12
        #pos=gpsCoordiante()
        leftSpeed=0
        rightSpeed= 0
        # Get the GPS co-ordinates
        x1 = - gps.getValues()[2]
        y1 = gps.getValues()[0]
        print(x1,y1)

        
        # if this cell in grid is a Dead-End then turn90 and move -forward
       
        
        """   
        if walltoRight() and count>20:
            
            message = struct.pack("cdd","X",x1,y1)
            emitter.send(message)
            
            
            #stop()
           
            while receiver.getQueueLength()>0:
                message=receiver.getData()
                print("TTT",message)
                dataList=struct.unpack("i",message)
                receiver.nextPacket()
                print("Pheromones",dataList[0])
                if dataList[0]==99:
                    print("Turning Right")
                    forward=0
                    count=0
                elif dataList[0]==98:
                    print("Turning Left")
                    step_little()
                    passive_wait(1.1)
                    turn90()
                    passive_wait(1.1)
                    forward=1
                    count=0
                elif dataList[0]==97:
                    print("Going Straight")
                    step_little()
                    passive_wait(1)
                    forward=1
                    count=0
                elif dataList[0]==96:
                    print("Going Back")
                    step_little()
                    passive_wait(1)
                    turn270()
                    passive_wait(1.1)
                    forward=1
                    count=0
                else:
                    stop()
                    print("Finished Instructing",dataList)
                    
            w_d2=0.7
            #eprev=(w_d2*ps[2].getValue() + (1-w_d2)*ps[1].getValue())- 80
            
        # If no cliff Detected then follow PID-Controller and send the supervisor a list["false-clif detected",x-coordinate,x-coordinate]
        # Supervisor fills the pheromoneGrid based on character sent over.   
        #ecurr = (w_d2*ps[2].getValue() + (1-w_d2)*ps[1].getValue())- 80
        
        message = struct.pack("cdd","F",x1,y1)
        emitter.send(message)
        # if this cell in grid is a Dead-End then turn90 and move -forward
        flag=0
        while receiver.getQueueLength()>0:
            message=receiver.getData()
            dataList=struct.unpack("i",message)
            receiver.nextPacket()
            print("Enter",dataList)
            # This Cell is Dead-End
            if dataList[0]>10:
                print("Dead-End,Turn 90")
                turn90()
                passive_wait(0.4)
                flag=1
        #follow wall, consider this square as rectangualr block
        
        # while the Robot is in the Dead-End cell move-straight
        while flag==1:
            print("GO-Back")
            leftSpeed=MAX_SPEED
            rightSpeed=MAX_SPEED
            if cliff_detected():
                leftSpeed=-MAX_SPEED
                rightSpeed=MAX_SPEED
            leftMotor.setVelocity(leftSpeed)
            rightMotor.setVelocity(rightSpeed)
            robot.step(TIME_STEP)
            #pos=gpsCoordiante()
            x = - gps.getValues()[2]
            y = gps.getValues()[0]
            message = struct.pack("cdd","F",x,y)
            emitter.send(message)
            while receiver.getQueueLength()>0:
                message=receiver.getData()
                dataList=struct.unpack("i",message)
                receiver.nextPacket()
                print("concentration: ",dataList)
                if dataList[0]>10:
                    print("Still inside ")
                else:
                    flag=0
                    
        """
        if cliff_detected():
            #Take turn
            #passive_wait(0.2)
            print("Cliff Detected")
            leftSpeed=-MAX_SPEED
            rightSpeed= MAX_SPEED
            forward=0
            #True cliff detected
            # Sends this event to the supervisor
            message = struct.pack("cdd","T",x1,y1)
            emitter.send(message)
            #leftMotor.setVelocity(leftSpeed)
            #rightMotor.setVelocity(rightSpeed)
            #passive_wait(0.2) 
        elif forward==1:
            print("FORWARD")
            leftSpeed=MAX_SPEED
            rightSpeed= MAX_SPEED
        else:
            # false cliff detected.
            print("Follow Wall")
            turn=1
            count+=1
            w_d2 = 0.7
            ecurr = (w_d2*ps[2].getValue() + (1-w_d2)*ps[1].getValue())- 80
            print("eeeeee",ecurr)
            ep = ecurr
            message = struct.pack("cdd","S",x1,y1)
            emitter.send(message)
            
            xs.append(ep)
            
            print("This",ep)
            ed = ecurr - eprev
            ei = ecurr + eprev
            eprev=ecurr
            Kp = 0.2
            Kd = 0.7
            Ki = 0.05
            sLeft = MAX_SPEED
            sRight = MAX_SPEED + (Kp*ep + Kd*ed + Ki*(ei))
            leftSpeed=sLeft
            rightSpeed= sRight
        
        # set speeds
        leftMotor.setVelocity(leftSpeed)
        rightMotor.setVelocity(rightSpeed)      
        
# calling solve_maze       
if __name__ == "__main__": 
    solve_maze()
