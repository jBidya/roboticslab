"""
Final Project
Roll:204101072
Email: jbidyanath@iitg.ac.in
Problem Description : Maze solver robot swarm (in webots)
Algorithm Used: wall-following, Dead-End Avoider, Cross-Road Explorer
--------------------------CONTROLLER PROGRAM FOR Supervisor Node----------------------
"""
# import Supervisor,essential library 
from controller import Supervisor
from controller import Receiver
import math
import random
import struct
import threading 
import pygeohash as gh
# time in [ms] of a simulation step
TIME_STEP=64
# create the Supervisor instance.
supervisor = Supervisor()
# get emitter with name "emitter"
# data is sent over the channel 4
emitter=supervisor.getEmitter("emitter")
# get emitter with name "emitter1"
# data is sent over the channel 2
emitter1=supervisor.getEmitter("emitter1")
# get receiver with name "receiver"
receiver=supervisor.getReceiver("receiver")
# receiver side buffer stores the data sent over for a period of TIME_STEP 
receiver.enable(TIME_STEP) 
# Listen to emitter at channel-1
receiver.setChannel(1)
# get receiver with name "receiver1"
receiver1=supervisor.getReceiver("receiver1")
# receiver side buffer stores the data sent over for a period of TIME_STEP 
receiver1.enable(TIME_STEP) 
# Listen to emitter at channel-3
receiver1.setChannel(3)
# This is a 12*12 Matrix used to store the pheromone concentration on each of the square of the check-board
pheromonegrid= [[0 for i in range(12)] for j in range(12)]
# this variable is used to store details about cross-overs encountered in the maze by all robots
cross_roads=dict()

# This function adds the location of the cross-over and the direction along which robots has already travelled
def addToGeohash(cross_roads, latitude, longitude,dir_left,dir_right,dir_back,dir_forward):
    p = (latitude, longitude,dir_left,dir_right,dir_back,dir_forward)
    ph = gh.encode(p[0],p[1],5)
    if ph not in m:
        cross_roads[ph] = []
    cross_roads[ph].append(p)
    return

 
# For given position/location of robot this function finds if the position is cross-over or not.
def getFromGeohash(m, latitude, longitude):
    p = (latitude, longitude)
    ph = gh.encode(p[0],p[1],5)
    print ("query hash: " + ph)
    if ph in m:
        return m[ph]
    return []
# This function translates the RAW-GPS co-ordinate of the robot into a cell of the matrix (12*12)
def gpsCoordiante(x,y):
    x1=math.floor(x/0.25)
    y1=math.floor(y/0.25)
    if x1<0:
        x1=-x1+5
    else:
        x1=5-x1
    if y1<0:
        y1=6-(-y1)
    else:
        y1=y1+6
    x1=int(x1)
    y1=int(y1)
    return [x1,y1]

# Thread for Robot-1
def robot_one():
    prev=[0,0]
    while supervisor.step(TIME_STEP) != -1:
        # Case 1: simulationReset
        # supervisor.simulationReset()
        """
        message = struct.pack("cii","x",1,1)
        emitter.send(message)
        """
        # Receive list from robot-1
        # Update Pheromone concentration at the location
        while receiver.getQueueLength()>0:
            message=receiver.getData()
            dataList=struct.unpack("cdd",message)
            receiver.nextPacket()
            # if not cliff detected
            print(dataList)
            x=dataList[1]
            y=dataList[2]
            # Convert GPS Co-ordinate to a cell of matrix
            pos=gpsCoordiante(x,y)
            print ("Length of hashmap: "+str(len(cross_roads)))
            #print ("Contents of map  : "+str(m))
            #print ("Query result     : ")
            # If Robot is moving "Straight"
            if dataList[0]=='S':
                if pheromonegrid[pos[0]][pos[1]]==0: 
                    pheromonegrid[pos[0]][pos[1]]=1
                #message = struct.pack("i",pheromonegrid[pos[0]][pos[1]])
                #emitter.send(message)
            # If robot Found a "Trap"
            elif dataList[0]=='T':
                if pheromonegrid[pos[0]][pos[1]]!=0: 
                    pheromonegrid[pos[0]][pos[1]]+=1   
            # Robot asking for pheromone concentration at the location
            elif dataList[0]=='F':
                message = struct.pack("i",pheromonegrid[pos[0]][pos[1]])
                emitter.send(message)     
            """
            elif dataList[0]=='S':
                if pheromonegrid[pos[0]][pos[1]]==0: 
                    pheromonegrid[pos[0]][pos[1]]=1
            elif dataList[0]=='X':   
                print("CHECK:",prev,pos)
                if pos[0]!=prev[0] or pos[1]!=prev[1]:
                    #
                    message = struct.pack("i",99)
                    emitter.send(message) 
                    #supervisor.step(TIME_STEP)
                    addToGeohash(cross_roads, x,y,0,1,0,0)
                    n=getFromGeohash(cross_roads,x,y)
                    print("Exist-1",n)
                    prev=pos
                    
                if pheromonegrid[pos[0]][pos[1]]==0: 
                    pheromonegrid[pos[0]][pos[1]]=-1   
            # if cliff detected
            elif dataList[0]=='T':
                if pheromonegrid[pos[0]][pos[1]]!=-1: 
                    pheromonegrid[pos[0]][pos[1]]+=1
            """
           
        #print(pheromonegrid)
        #print(m)
        # Robot-2 is getting the pheromone concentration at the location
        while receiver1.getQueueLength()>0:
            message=receiver1.getData()
            dataList=struct.unpack("cdd",message)
            receiver1.nextPacket()
            # if not cliff detected
            #print(dataList)
            x=dataList[1]
            y=dataList[2]
            pos=gpsCoordiante(x,y)
            n=getFromGeohash(cross_roads,x,y)
            print("Exist-2",n)
            c=len(n)
            # Robot-2 asking for pheromone concentration
            if dataList[0]=="F":
                print("ARRIVED-F")
                message = struct.pack("i",pheromonegrid[pos[0]][pos[1]])
                emitter1.send(message)
# Thread for Robot-2
def robot_two():
    prev=[0,0]
    while supervisor.step(TIME_STEP) != -1:
        # Case 1: simulationReset
        # supervisor.simulationReset()
        """
        message = struct.pack("cii","x",1,1)
        emitter.send(message)
        """
        # Receive list from robot-1
        # Update Pheromone concentration at the location
        while receiver1.getQueueLength()>0:
            message=receiver1.getData()
            dataList=struct.unpack("cdd",message)
            receiver1.nextPacket()
            # if not cliff detected
            #print(dataList)
            x=dataList[1]
            y=dataList[2]
            pos=gpsCoordiante(x,y)
            n=getFromGeohash(cross_roads,x,y)
            print("Exist-2",n)
            c=len(n)
            """
            if c>0:
                print("thread-2",n)
                l=n[0][2]
                r=n[0][3]
                f=n[0][4]
                b=n[0][5]
                if l==0:
                    message = struct.pack("i",98)
                    emitter1.send(message)
                elif r==0:
                    message = struct.pack("i",99)
                    emitter1.send(message)
                elif f==0:                           
                    message = struct.pack("i",97)
                    emitter1.send(message)
                elif b==0:
                    message = struct.pack("i",96)
                    emitter1.send(message)    
                #supervisor.step(TIME_STEP)
            # Add 2 points in general area (1st near vista del mar)
           """
            if dataList[0]=="F":
                print("ARRIVED-F")
                message = struct.pack("i",pheromonegrid[pos[0]][pos[1]])
                emitter1.send(message)
                    
if __name__ == "__main__": 
    robot_one()
    """
    t1 = threading.Thread(target=robot_one, name='t1') 
    t2 = threading.Thread(target=robot_two, name='t2')   
      
    print("threads made")
          
    # starting threads 
    t1.start() 
    t2.start() 
      
    print("threads started")
          
    # wait until all threads finish 
    t1.join() 
    t2.join()
    
    print("threads finished")
    """
    