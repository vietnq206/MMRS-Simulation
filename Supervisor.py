

from turtle import width
import pygame
import math
import numpy as np
from dijkstra_search import DijkstraSearch
from pathPlaning import *
import json
from enum import Enum





#Map init
 
wScreen = 800
hScreen = 800
numGridX = 20
numGridY = 20
sectorSize = int(wScreen/20)

speedMax = 20
timeStep = 0.01
robotRadius = 6

############## State of the robot
st_STOP = 0
st_RUN = 1
st_ASK = 2
st_DONE = 3 # when it done the job

########### level of priority
num_level = 5




def get_angle(vec):
    angle =  math.atan2(vec[1], vec[0])
    while (angle < -np.pi or angle > np.pi):
        if ( angle < -np.pi):
            angle += 2*np.pi
        else:
            angle -= 2*np.pi
    
    return angle

class Supervisor:
    def __init__(self,robots):
        self.robots = robots
        self.MapToken = np.zeros( (numGridX,numGridY) , dtype=np.int64) - 1
        self.List_order = list()
        for idx in range(len(self.robots)):
            if self.MapToken[self.robots[idx].path[0][0]][self.robots[idx].path[0][1]] == -1:
                self.MapToken[self.robots[idx].path[0][0]][self.robots[idx].path[0][1]] = idx
    
    def release_register(self):
        for idx in range(len(self.robots)):
            if(self.robots[idx].release_prevNode == 1):
               self.MapToken[self.robots[idx].release_node()[0]][self.robots[idx].release_node()[1]] = -1  
               self.robots[idx].release_prevNode = 0     

    def ask_register(self):

        list_robot_ask = [list()]*len(self.robots) # using to manage priority of robots

        for idx in range(len(self.robots)):    
            if self.robots[idx].state == st_ASK:
                if self.MapToken[self.robots[idx].asking_node()[0]][self.robots[idx].asking_node()[1]] == -1:
                    if [self.robots[idx].asking_node()[0],self.robots[idx].asking_node()[1]] in list_robot_ask:
                        tmpIdx = list_robot_ask.index([self.robots[idx].asking_node()[0],self.robots[idx].asking_node()[1]])
                        if (self.robots[tmpIdx].priority_level < self.robots[idx].priority_level):
                            list_robot_ask[tmpIdx] = list()
                            list_robot_ask[idx] = [self.robots[idx].asking_node()[0],self.robots[idx].asking_node()[1]]
                    else:
                        list_robot_ask[idx] = [self.robots[idx].asking_node()[0],self.robots[idx].asking_node()[1]]

        for idx in range(len(list_robot_ask)):
            if len(list_robot_ask[idx]) > 0:
                if (self.MapToken[list_robot_ask[idx][0]][list_robot_ask[idx][1]] == -1 and self.robots[idx].get_state() == st_ASK):
                    self.MapToken[list_robot_ask[idx][0]][list_robot_ask[idx][1]] = idx
                    self.robots[idx].request_accepted()        
                

    def check_collision(self):
        for i in range(len(self.robots)):
            ack = 0
            for j in range(len(self.robots)):
                if i != j:
                    if  np.sqrt((self.robots[i].x - self.robots[j].x)**2 + (self.robots[i].y - self.robots[j].y)**2 ) <= robotRadius:
                        ack = 1
                        self.robots[i].collision = True
                        self.robots[j].collision = True
            if ack == 0:
                self.robots[i].collision = False
                
    def import_task(self,path):
        with open(path, 'r') as f:
            self.List_order = json.load(f)['Sheet1']

    


    def print_register_map(self):
        print("Map all:")
        for i in range(numGridX):
            s = ""
            for j in range(numGridY):
                s = s + str(self.MapToken[i][j]) +" "
            print(s)

    
class robot(object):
    def __init__(self,nodeX,nodeY,radius,color):
        self.loc_node_x = nodeX
        self.loc_node_y = nodeY
        self.x = nodeX*sectorSize
        self.y = nodeY*sectorSize
        self.speed = 0
        self.orient = 0
        self.radius = radius
        self.color = color
        self.collision = False
        self.release_prevNode = -1
        self.path = []

        self.indexPath = 0
        self.state = st_ASK
        self.priority_level = 0

    def draw(self, win):
        pygame.draw.circle(win, (0,0,0), (self.x,self.y), self.radius)
        if self.collision:
            pygame.draw.circle(win, (255,0,0), (self.x,self.y), self.radius-1)
        else:
            pygame.draw.circle(win, self.color, (self.x,self.y), self.radius-1)
    
    def release_node(self):
        return self.path[self.indexPath-1]
    def asking_node(self):
        if ( self.indexPath == len(self.path)-1):
            self.state = st_DONE
            return self.path[self.indexPath]
        return self.path[self.indexPath+1]    
    def request_accepted(self):
        if ( self.indexPath == len(self.path)-1):
            self.state = st_DONE
        else:
            self.state =  st_RUN
            self.indexPath += 1

    def get_state(self):
        return self.state

    def move(self,speed):
        if self.state == st_RUN:   
            #Test if the robot has achived  haft of the sector
            middle_point = (self.path[self.indexPath-1] + self.path[self.indexPath])*sectorSize/2
            gap = sectorSize/14
            if ( self.x > middle_point[0] - gap and self.x < middle_point[0] + gap) and  ( self.y > middle_point[1] - gap and self.y < middle_point[1] + gap) and self.release_prevNode == -1:
                self.release_prevNode = 1                

            # runnign
            self.orient = get_angle(self.path[self.indexPath]*sectorSize - [self.x,self.y])
            self.speed = speed
            velx = math.cos(self.orient) * self.speed
            vely = math.sin(self.orient) * self.speed
            self.x = velx*timeStep + self.x 
            self.y = vely*timeStep + self.y




        

    def reachNodePath(self):
        # if( self.indexPath == len(self.path)-1):
        #     self.state = st_DONE
        if ( np.abs(np.linalg.norm([self.x,self.y] - self.path[self.indexPath]*sectorSize )) <= sectorSize/100) and self.state == st_RUN:
            self.release_prevNode = -1
            self.state = st_ASK
            

    def hitBoundaries(self):
        if any( (self.x <= self.radius,self.y <= self.radius,self.x >= wScreen - self.radius,self.y >= hScreen - self.radius)):
            return True
        else:
            return False
    def pathAssign(self,path):
        [X,Y] = path
        self.x = X[0]*sectorSize
        self.y = Y[0]*sectorSize
        for x,y in zip( X,Y):
            self.path.append(np.array([x,y]))


    @staticmethod
    def ballPath(startx, starty, power, ang, time):
        angle = ang
        velx = math.cos(angle) * power
        vely = math.sin(angle) * power

        distX = velx * time
        distY = (vely * time) 

        newx = round(distX + startx)
        newy = round(starty - distY)


        return (newx, newy)


class obstacles:
    def __init__(self,x,y,heigh,width,color):
        #geometry initilization
        self.x = x
        self.y = y
        self.heigh = heigh
        self.width = width
        self.color = color

        self.nodes = []
        for row in range(self.x,self.x+self.width+1):
            for col in range(self.y,self.y + self.heigh +1):
                self.nodes.append((row,col))


    def draw(self,screen):
        pygame.draw.rect(screen, self.color, pygame.Rect(self.x*sectorSize, self.y*sectorSize, self.width*sectorSize, self.heigh*sectorSize))
        for elm in self.nodes:
            pygame.draw.circle(screen, (255,0,0), elm, 2)

    def getHit(self,rb):
        if (
            ( (np.abs(rb.x-self.x*sectorSize)<=rb.radius or np.abs(rb.x-self.x*sectorSize - self.width*sectorSize)<=rb.radius)  and rb.y > self.y*sectorSize and rb.y < self.y*sectorSize + self.heigh*sectorSize)  or \
            ( (np.abs(rb.y-self.y*sectorSize)<=rb.radius or np.abs(rb.y-self.y*sectorSize - self.heigh*sectorSize)<=rb.radius ) and rb.x > self.x*sectorSize and rb.x < self.x*sectorSize + self.width*sectorSize) or  \
            (math.sqrt((rb.x-self.x*sectorSize)**2 + (rb.y  - self.y*sectorSize)**2) < rb.radius ) or \
            (math.sqrt((rb.x-self.x*sectorSize-self.width*sectorSize)**2 + (rb.y  - self.y*sectorSize - self.heigh*sectorSize)**2) < rb.radius ) or \
            (math.sqrt((rb.x-self.x*sectorSize)**2 + (rb.y  - self.y*sectorSize - self.heigh*sectorSize)**2) < rb.radius ) or \
            (math.sqrt((rb.x-self.x*sectorSize-self.width*sectorSize)**2 + (rb.y  - self.y*sectorSize)**2) < rb.radius ) ):
            
            return True
        else:
            return False
 