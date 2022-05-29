

from turtle import width
import pygame
import math
import numpy as np
from dijkstra_search import DijkstraSearch
from pathPlaning import *
import json
from enum import Enum
from readfile import *




#Map init
 
wScreen = 800
hScreen = 800
numGridX = 20
numGridY = 20
sectorSize = int(wScreen/20)

speedMax = 20
timeStep = 0.01
robotRadius = 7

############## State of the robot
st_STOP = 0
st_RUN = 1
st_ASK = 2
st_DONE = 3 # when it done the job

########### level of priority
num_level = 5

########## Map infor
G_loc_package_load = [[4,7],[4,10],[6,7],[6,10],[9,7],[9,10],[11,7],[11,10],[14,7],[14,10],[16,7],[16,10],[5,14],[8,14],[5,17],[8,17],[13,14],[16,14],[13,17],[16,17]]
G_loc_outport = [[6,1],[11,1],[16,1]]










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
        self.MapToken = np.zeros( (numGridX*2,numGridY*2) , dtype=np.int64) - 1
        self.Index_order = 0
        self.List_order = list()
        self.Order_done = list()
        for idx in range(len(self.robots)):
            if self.MapToken[int(self.robots[idx].loc_node_x*2)][int(self.robots[idx].loc_node_y*2)] == -1:
                self.MapToken[int(self.robots[idx].loc_node_x*2)][int(self.robots[idx].loc_node_y*2)] = idx
    
    def release_register(self):
        for idx in range(len(self.robots)):
            if(self.robots[idx].release_prevNode == 1):
               self.MapToken[int(self.robots[idx].release_node()[0]*2)][int(self.robots[idx].release_node()[1]*2)] = -1  
               self.robots[idx].release_prevNode = 0     

    def deadlock_detection(self,rb):
        # print("In"+str(rb))
        i_node = self.robots[rb].node_deadlock()
        i_rb = self.MapToken[int(i_node[0]*2)][int(i_node[1]*2)]
        if i_rb == rb:
            return True
        c = 0
        while ( i_rb != -1 ):
            # print("robot:"+str(i_rb))
            if self.robots[i_rb].state == st_RUN and self.robots[i_rb].release_prevNode == -1 :
                return True
            if self.robots[i_rb].release_prevNode == 1:
                n_node = self.robots[i_rb].node_deadlock()
            else:
                n_node = self.robots[i_rb].asking_node()
            if n_node[0] == self.robots[rb].asking_node()[0] and n_node[1] == self.robots[rb].asking_node()[1]:
                return False
            i_rb = self.MapToken[int(n_node[0]*2)][int(n_node[1]*2)]
            if rb == i_rb:
                return True
            c += 1
            if c == 10:
                input()
        
        return True

    def ask_register(self):

        list_robot_ask = [list()]*len(self.robots) # using to manage priority of robots
        
        for idx in range(len(self.robots)):    
 
            # print("Robot num: "+str(idx)+" with status "+str(self.robots[idx].state)+" next node ["+ str(self.robots[idx].asking_node()[0])+","+str(self.robots[idx].asking_node()[1])\
            #             +"] and next more node ["+ str(self.robots[idx].node_deadlock()[0])+","+str(self.robots[idx].node_deadlock()[1]) +"]")
            if self.deadlock_detection(idx): 
                # print("Out")   
                if self.robots[idx].state == st_ASK:
                    
                    askNodeX = int(self.robots[idx].asking_node()[0]*2)
                    askNodeY = int(self.robots[idx].asking_node()[1]*2)
                    # print("State token: "+str(self.MapToken[askNodeX][askNodeY]))
                    # input()
                    if self.MapToken[askNodeX][askNodeY] == -1:
                        if [askNodeX,askNodeY] in list_robot_ask:
                            tmpIdx = list_robot_ask.index([askNodeX,askNodeY])
                            if (self.robots[tmpIdx].priority_level < self.robots[idx].priority_level):
                                list_robot_ask[tmpIdx] = list()
                                list_robot_ask[idx] = [askNodeX,askNodeY]
                        else:
                            list_robot_ask[idx] = [askNodeX,askNodeY]
                         
                                                                                                                   

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
                
    def import_task(self):
        self.List_order = list_task['task']
        for elm in self.List_order:
            elm['Done'] = 0

 

    def generate_path(self,rbIdx,access_nodes):
        node_seq = [[self.robots[rbIdx].loc_node_x,self.robots[rbIdx].loc_node_y]]
        o_path = list()
        order = self.Index_order
        for i in range(len(self.List_order[order]['Pack_list'])):
            if(self.List_order[order]['Pack_list'][i] == '1'):
                node_seq.append(G_loc_package_load[i])

        if ( self.List_order[order]['Out_port'] == 'A'):
            node_seq.append(G_loc_outport[0])
        elif ( self.List_order[order]['Out_port'] == 'B'):
            node_seq.append(G_loc_outport[1])
        else:
            node_seq.append(G_loc_outport[2])

        o_path.append(np.array([node_seq[0][0],node_seq[0][1]]))
        
        for i in range(len(node_seq)-1):
            rx,ry = VisibilityRoadMap(robotRadius, do_plot=False)\
                                    .planning(node_seq[i][0] ,node_seq[i][1], node_seq[i+1][0], node_seq[i+1][1], access_nodes)
            for x,y in zip( rx,ry): 
                if o_path[-1][0] != x or o_path[-1][1] != y: 
                    o_path.append(np.array([x,y]))
             

        self.Index_order += 1
        return o_path

    

    def gen_Path(self,rbIdx,access_nodes,rep):
        if ( self.Index_order == 10 ):
            self.Index_order = 0
        node_seq = [[self.robots[rbIdx].loc_node_x,self.robots[rbIdx].loc_node_y]]
        task = list_task_2['task'][self.Index_order]
        o_path = list()
        for elm in task['NumOrder']:
            node_seq.append(G_loc_package_load[elm])
        node_seq.append(G_loc_outport[task['port']])
        node_seq.append(node_seq[0])

        self.Index_order +=1
        o_path.append(np.array([node_seq[0][0],node_seq[0][1]]))
        for i in range(len(node_seq)-1):
            rx,ry = VisibilityRoadMap(robotRadius, do_plot=False)\
                                    .planning(node_seq[i][0] ,node_seq[i][1], node_seq[i+1][0], node_seq[i+1][1], access_nodes)
            for x,y in zip( rx,ry): 
                if o_path[-1][0] != x or o_path[-1][1] != y: 
                    o_path.append(np.array([x,y]))

        for i in range(rep):
            o_path.extend(o_path[1:len(o_path)])

        return o_path


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
    
    def node_deadlock(self):
        return self.path[self.indexPath+2]    

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
            loc_error = math.sqrt((self.path[self.indexPath-1][0]*sectorSize- self.x)**2 + (self.path[self.indexPath-1][1]*sectorSize- self.y)**2)

            # gap = sectorSize/14
            # if ( self.x > middle_point[0] - gap and self.x < middle_point[0] + gap) and  ( self.y > middle_point[1] - gap and self.y < middle_point[1] + gap) and self.release_prevNode == -1:
            #     self.release_prevNode = 1        

            if ( loc_error > 2*robotRadius) and self.release_prevNode == -1:
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
        self.x = x*sectorSize + 2*robotRadius
        self.y = y*sectorSize + 2*robotRadius
        self.heigh = heigh*sectorSize-4*robotRadius
        self.width = width*sectorSize-4*robotRadius
        self.color = color

        self.nodes = []
        for row in range(x+1,x+width):
            for col in range(y+1, y +  heigh ):
                self.nodes.append((row,col))

        for row in range(x,x+width):
            for col in range(y, y +  heigh ):
                self.nodes.append((row+0.5,col+0.5))


        # self.vertexX = x*sectorSize + 2*robotRadius
        # self.vertexY = y*sectorSize + 2*robotRadius


    def draw(self,screen):


        pygame.draw.rect(screen, self.color, pygame.Rect(self.x , self.y , self.width , self.heigh ))
        for elm in self.nodes:
            pygame.draw.circle(screen, (255,0,0), elm, 2)

    def getHit(self,rb):
        if (
            ( (np.abs(rb.x-self.x )<=rb.radius or np.abs(rb.x-self.x  - self.width)<=rb.radius)  and rb.y > self.y and rb.y < self.y + self.heigh)  or \
            ( (np.abs(rb.y-self.y)<=rb.radius or np.abs(rb.y-self.y - self.heigh)<=rb.radius ) and rb.x > self.x  and rb.x < self.x  + self.width) or  \
            (math.sqrt((rb.x-self.x )**2 + (rb.y  - self.y)**2) < rb.radius ) or \
            (math.sqrt((rb.x-self.x -self.width)**2 + (rb.y  - self.y - self.heigh)**2) < rb.radius ) or \
            (math.sqrt((rb.x-self.x )**2 + (rb.y  - self.y - self.heigh)**2) < rb.radius ) or \
            (math.sqrt((rb.x-self.x -self.width)**2 + (rb.y  - self.y)**2) < rb.radius ) ):
            
            return True
        else:
            return False
 