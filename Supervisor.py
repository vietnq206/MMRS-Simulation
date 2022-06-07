

from ast import While
from msilib import Directory
from turtle import width
import pygame
import math
import numpy as np
from dijkstra_search import DijkstraSearch
from pathPlaning import *
import json
from enum import Enum
from readfile import *


verSim = 1

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
st_DONE = 3 
st_WAIT = 4

# when it done the job

########### level of priority
num_level = 6

########## Map infor
G_loc_package_load = [[4,7],[4,10],[6,7],[6,10],[9,7],[9,10],[11,7],[11,10],[14,7],[14,10],[16,7],[16,10],[5,14],[8,14],[5,17],[8,17],[13,14],[16,14],[13,17],[16,17]]
G_loc_outport = [[6,1],[11,1],[16,1]]

def getList(dict):
    tmp = set()
    for elm in dict.keys():
        tmp.add(elm)
    return tmp

def direction(p1,p2):
    if p1[0] == p2 [0]:
        if p2[1] > p1[1]:
            return 4
        else:
            return 0
    if p2[1] == p1[1]:
        if p2[0] > p1[1]:
            return 6
        else:
            return 2
    if p2[0] > p1[0]:
        if p2[1] > p1[1]:
            return 5
        else:
            return 7

    if p2[0] < p1[0]:
        if p2[1] > p1[1]:
            return 3
        else:
            return 1


def list_compare(a,b):
    a.sort()
    b.sort()
    if a == b:
        return True
    else:
        return False
def find_adj(a,b):
    out = list()
    if (a[0] - int(a[0]) == 0):
        if (a[0],a[1]-1)  in b:
            out.append([a[0],a[1]-1])

        if (a[0],a[1]+1)  in b:
            out.append([a[0],a[1]+1])

        if (a[0]-1,a[1])  in b:
            out.append([a[0]-1,a[1]])

        if (a[0]+1,a[1])  in b:
            out.append([a[0]+1,a[1]])

    
    if (a[0]-0.5,a[1]-0.5)  in b:
        out.append([a[0]-0.5,a[1]-0.5])

    if (a[0]+0.5,a[1]+0.5)  in b:
        out.append([a[0]+0.5,a[1]+0.5])

    if (a[0]-0.5,a[1]+0.5)  in b:
        out.append([a[0]-0.5,a[1]+0.5])

    if (a[0]+0.5,a[1]-0.5)  in b:
        out.append([a[0]+0.5,a[1]-0.5])

    return out







def get_angle(vec):
    angle =  math.atan2(vec[1], vec[0])
    while (angle < -np.pi or angle > np.pi):
        if ( angle < -np.pi):
            angle += 2*np.pi
        else:
            angle -= 2*np.pi
    
    return angle

class Supervisor:
    def __init__(self,robots,access_nodes):
        self.access_nodes = access_nodes
        self.robots = robots
        self.MapToken = np.zeros( (numGridX*2,numGridY*2) , dtype=np.int64) - 1
        self.Index_order = 0
        self.List_order = list()
        self.Order_done = list()
        self.robots_alloc = [list()]*numGridX*2 # using to manage priority of robots

        self.dict_directRobots = dict()

        for idx in range(len(self.robots)):
            if self.MapToken[int(self.robots[idx].loc_node_x*2)][int(self.robots[idx].loc_node_y*2)] == -1:
                self.MapToken[int(self.robots[idx].loc_node_x*2)][int(self.robots[idx].loc_node_y*2)] = idx
    
    def release_register(self):
        for idx in range(len(self.robots)):
            if(self.robots[idx].release_prevNode == 1):
                # for i in range(numGridX*2):
                #     for j in range(numGridY*2):
                #         if i != self.robots[idx].path[self.robots[idx].indexPath][0] and j != self.robots[idx].path[self.robots[idx].indexPath][1] :
                            
                #             if self.MapToken[i][j] == idx:
                #                 self.MapToken[i][j]  = -1    
                 


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

    def push_deadlock(self,rb): 
        # print("In"+str(rb))
        i_node = self.robots[rb].asking_node()
        i_rb = self.MapToken[int(i_node[0]*2)][int(i_node[1]*2)]
        # if i_rb == rb:
        #     return True
        c = 0
        cyclic_deadlock = list()
        cyclic_deadlock.append(rb)
        while ( i_rb != -1 ):
            # print("robot:"+str(i_rb))
            cyclic_deadlock.append(i_rb)
            if self.robots[i_rb].state == st_RUN and self.robots[i_rb].release_prevNode == -1 :
                return list()
            # if self.robots[i_rb].release_prevNode == 1:
            #     n_node = self.robots[i_rb].node_deadlock()
            # else:
            #     n_node = self.robots[i_rb].asking_node()
            n_node = self.robots[i_rb].asking_node()
            if n_node[0] == self.robots[rb].asking_node()[0] and n_node[1] == self.robots[rb].asking_node()[1]:
                return list()
            i_rb = self.MapToken[int(n_node[0]*2)][int(n_node[1]*2)]
            if rb == i_rb:
                return cyclic_deadlock
            c += 1
            if c == 10:
                input()
                return list()
        
        return list()

    def checkTrap(self,path,rb):
        TmpKey = str(path[0][0])+":"+str(path[0][1])     
        setRb = getList(self.dict_directRobots[TmpKey])
        setRb.remove(rb)
        i = 1
        while len(setRb)!=0:
            direct = direction(path[i],path[i-1])
            TmpKey = str(path[i][0])+":"+str(path[i][1])     
            setRb = setRb.intersection(getList(self.dict_directRobots[TmpKey])) 
            rmRb = set()
            for elm in setRb:
                if direct in self.dict_directRobots[TmpKey][elm] and self.MapToken[int(path[i][0]*2)][int(path[i][1]*2)] == elm:
                    # if self.robots[elm].state == st_ASK :
                    if    direct == direction(self.robots[elm].path[self.robots[elm].indexPath],self.robots[elm].path[self.robots[elm].indexPath+1]):
                        return False

                    # if self.robots[elm].state == st_RUN and self.robots[elm].release_prevNode != -1 :
                    #     tmp_direct = direction(self.robots[elm].path[self.robots[elm].indexPath-1],self.robots[elm].path[self.robots[elm].indexPath])  

                    # if direct == tmp_direct:    
                    #     return False
                if direct not in self.dict_directRobots[TmpKey][elm]:
                    rmRb.add(elm)


            
            if (len(rmRb)!=0):
                for elm in rmRb:
                    setRb.remove(elm)
            i +=1

        return True



    def ask_register(self):
        list_robot_ask = [list()]*len(self.robots) # using to manage priority of robots  
        if verSim ==  0:
            
            for idx in range(len(self.robots)):    
                # print("Robot num: "+str(idx)+" with status "+str(self.robots[idx].state)+" next node ["+ str(self.robots[idx].asking_node()[0])+","+str(self.robots[idx].asking_node()[1])\
                #             +"] and next more node ["+ str(self.robots[idx].node_deadlock()[0])+","+str(self.robots[idx].node_deadlock()[1]) +"]")
                if self.deadlock_detection(idx):   
                    if self.robots[idx].state == st_ASK:
                        
                        askNodeX = int(self.robots[idx].asking_node()[0]*2)
                        askNodeY = int(self.robots[idx].asking_node()[1]*2)

                        if self.MapToken[askNodeX][askNodeY] == -1:

                            unexe_nodes = self.robots[idx].unexe_nodes()
                            if (self.checkTrap(unexe_nodes,idx)):

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

        if verSim == 1:
            assigned_rb = list()
            for idx in range(len(self.robots)):    
                if self.robots[idx].state == st_ASK:
                    for x in range(numGridX*2):
                        for y in range(numGridX*2):
                            if x != int(2*self.robots[idx].path[self.robots[idx].indexPath][0]) and y != int(2*self.robots[idx].path[self.robots[idx].indexPath][1]) and self.MapToken[x][y] == idx:
                                self.MapToken[x][y] = -1
                                
                    # if self.MapToken[int(2*(self.robots[idx].path[self.robots[idx].indexPath-1][0]))][int(2*(self.robots[idx].path[self.robots[idx].indexPath-1][1]))] == idx:
                    #     self.MapToken[int(2*(self.robots[idx].path[self.robots[idx].indexPath-1][0]))][int(2*(self.robots[idx].path[self.robots[idx].indexPath-1][1]))] = -1
                    askNodeX = int(self.robots[idx].asking_node()[0]*2)
                    askNodeY = int(self.robots[idx].asking_node()[1]*2)
                    if self.MapToken[askNodeX][askNodeY] == -1:
                        self.MapToken[askNodeX][askNodeY] = idx
                        assigned_rb.append(idx)
                        self.robots[idx].request_accepted()  

                        # if [askNodeX,askNodeY] in list_robot_ask:
                        #     tmpIdx = list_robot_ask.index([askNodeX,askNodeY])
                        #     if (self.robots[tmpIdx].priority_level < self.robots[idx].priority_level):
                        #         list_robot_ask[tmpIdx] = list()
                        #         list_robot_ask[idx] = [askNodeX,askNodeY]
                        # else:
                        #     list_robot_ask[idx] = [askNodeX,askNodeY]
                            
                                                                                                                    
            
            # for idx in range(len(list_robot_ask)):
            #     if len(list_robot_ask[idx]) > 0:
            #         if (self.MapToken[list_robot_ask[idx][0]][list_robot_ask[idx][1]] == -1 and self.robots[idx].get_state() == st_ASK):
            #             self.MapToken[list_robot_ask[idx][0]][list_robot_ask[idx][1]] = idx
            #             # if idx == 12:
            #             #     print("1")
            #             #     input()
            #             assigned_rb.append(idx)
            #             self.robots[idx].request_accepted()  

            flag = 0
            for idx in range(len(self.robots)):
                if self.robots[idx].state == st_ASK and flag == 0 :
                    cyc = self.push_deadlock(idx)
                    if len(cyc) != 0:
                        print(cyc)
                        print("Robot num: "+str(idx)+" with status "+str(self.robots[idx].state)+" next node ["+ str(self.robots[idx].curr_node()[0])+","+str(self.robots[idx].curr_node()[1]))
                        adj = find_adj(self.robots[idx].curr_node(),self.access_nodes)               
                        for elm in adj:
                            if  self.MapToken[int(elm[0])*2][int(elm[1])*2] == -1 and flag== 0:
                                # print("Robot num: "+str(idx)+" with status "+str(self.robots[idx].state)+" next node ["+ str(self.robots[idx].curr_node()[0])+","+str(self.robots[idx].curr_node()[1]))
                                self.robots[idx].path.insert(self.robots[idx].indexPath+1, self.robots[idx].curr_node())
                                self.robots[idx].path.insert(self.robots[idx].indexPath+1,np.array([elm[0],elm[1]]))
                                self.MapToken[int(elm[0])*2][int(elm[1])*2] = idx
                                self.robots[idx].spotlight = True
                                self.robots[idx].countPush += 1
                                self.robots[idx].request_accepted() 
                                flag = 1
                                break                        

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

 

    def generate_path_sigle(self,rbIdx):
        if ( self.Index_order == 10 ): #index to the task
            self.Index_order = 0
        if len(self.robots[rbIdx].task) == 0:
            task = list_task_2['task'][self.Index_order]
            node_seq = list()
            for elm in task['NumOrder']:
                node_seq.append(G_loc_package_load[elm])
            node_seq.append(G_loc_outport[task['port']])
            self.robots[rbIdx].task = node_seq
              


    # if self.robots[rbIdx].state == st_WAIT:
        #     if self.robots[rbIdx].indexTask == -1:
        #         self.robots[rbIdx].indexTask += 1
        #         node_seq = [[self.robots[rbIdx].curr_node()[0],self.robots[rbIdx].curr_node()[1]]]
        #         task = list_task_2['task'][self.Index_order]
        #         o_path = list()
        #         rx,ry = VisibilityRoadMap(robotRadius, do_plot=False)\
        #                             .planning(node_seq[0][0] ,node_seq[0][1], task[self.robots[rbIdx].indexTask][0], task[self.robots[rbIdx].indexTask][1], self.access_nodes)            
        #         for x,y in zip( rx,ry): 
        #             o_path.append(np.array([x,y]))

        #         self.Index_order += 1
        #         return o_path
        #     else:


    

    def gen_Path(self,rbIdx,rep):
        if ( self.Index_order == 10 ): #index to the task
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
                                    .planning(node_seq[i][0] ,node_seq[i][1], node_seq[i+1][0], node_seq[i+1][1], self.access_nodes)
            for x,y in zip( rx,ry): 
                if o_path[-1][0] != x or o_path[-1][1] != y: 
                    o_path.append(np.array([x,y]))

        for i in range(len(o_path)-1):       #for deadlock avoidance
            direct = direction(o_path[i],o_path[i+1])
            key = str(o_path[i][0])+":"+str(o_path[i][1])
            if key not in self.dict_directRobots:
                self.dict_directRobots[key] = {rbIdx:[direct]}
            else:
                if rbIdx in self.dict_directRobots[key]:
                    if direct not in self.dict_directRobots[key][rbIdx]:
                        self.dict_directRobots[key][rbIdx].append(direct)
                else:
                    self.dict_directRobots[key][rbIdx] = [direct]
                    
        for i in range(rep):
            o_path.extend(o_path[1:len(o_path)])

        # for i in range(len(o_path)):


        return o_path


    def print_register_map(self):
        print("Map all:")
        for i in range(numGridX*2):
            s = ""
            for j in range(numGridY*2):
                s = s + str(self.MapToken[i][j]) +" "
            print(s)

    
class robot(object):
    def __init__(self,nodeX,nodeY,radius,speed,color):
        self.loc_node_x = nodeX
        self.loc_node_y = nodeY
        self.x = nodeX*sectorSize
        self.y = nodeY*sectorSize
        self.speed = speed
        self.orient = 0
        self.radius = radius
        self.color = color
        self.collision = False
        self.release_prevNode = -1
        self.path = []
        self.spotlight = False
        self.indexPath = 0
        self.state = st_ASK
        self.priority_level = 0

        self.countPush = 0

        self.task = list()
        self.indexTask = -1

    def draw(self, win):
        if self.spotlight :
            pygame.draw.circle(win, (255,0,0), (self.x,self.y), self.radius-1)
        else:        
            if self.state != st_DONE:    
                pygame.draw.circle(win, (0,0,0), (self.x,self.y), self.radius)
                if self.collision:
                    pygame.draw.circle(win, (255,0,0), (self.x,self.y), self.radius-1)
                else:
                    pygame.draw.circle(win, self.color, (self.x,self.y), self.radius-1)
    
    def node_deadlock(self):
        if ( self.indexPath == len(self.path)-2):   
            self.state = st_DONE
            return self.path[self.indexPath]

        return self.path[self.indexPath+2]    
    def curr_node(self):
        if len(self.path) == 0:
            return np.array([self.loc_node_x,self.loc_node_y])
        return self.path[self.indexPath] 
    def release_node(self):
        return self.path[self.indexPath-1]
    def asking_node(self):
        if ( self.indexPath == len(self.path)-1):
            self.state = st_DONE
            return self.path[self.indexPath]
        return self.path[self.indexPath+1]    

    def unexe_nodes(self):
        return self.path[self.indexPath+1:len(self.path)] 
    
    def request_accepted(self):
        if ( self.indexPath == len(self.path)-1):
            self.state = st_DONE
        else:
            self.state =  st_RUN
            self.indexPath += 1

    def get_state(self):
        return self.state

    def move(self):
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
            # self.speed = speed
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
            if self.countPush != 0:
                self.countPush -= 1
            else: 
                self.spotlight = False
            

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
 