from turtle import width
import pygame
import math
import numpy as np
from dijkstra_search import DijkstraSearch
from pathPlaning import *
from Supervisor import *
#Map init


screen= pygame.display.set_mode((wScreen,hScreen))




def get_angle(vecX,vecY):
    angle =  math.atan2(vecY, vecX)
    while (angle < -np.pi or angle > np.pi):
        if ( angle < -np.pi):
            angle += 2*np.pi
        else:
            angle -= 2*np.pi
    
    return angle


class robot(object):
    def __init__(self,x,y,radius,color):
        self.x = x
        self.y = y
        self.speed = 0
        self.orient = 0
        self.radius = radius
        self.color = color

        
        self.pathX = []
        self.pathY = []
        self.indexPath = 1

    def draw(self, win):
        pygame.draw.circle(win, (0,0,0), (self.x,self.y), self.radius)
        pygame.draw.circle(win, self.color, (self.x,self.y), self.radius-1)
        


    def move(self,speed):
        self.orient = get_angle(self.pathX[self.indexPath]*sectorSize-self.x,self.pathY[self.indexPath]*sectorSize - self.y)
        self.speed = speed
        velx = math.cos(self.orient) * self.speed
        vely = math.sin(self.orient) * self.speed
        self.x = velx*timeStep + self.x 
        self.y = vely*timeStep + self.y

        

    def reachNodePath(self):
        if ( np.abs(np.linalg.norm(np.array([self.x,self.y])-np.array([self.pathX[self.indexPath]*sectorSize,self.pathY[self.indexPath]*sectorSize]))) <= 0.1):
            return True
        else:
            return False



    def hitBoundaries(self):
        if any( (self.x <= self.radius,self.y <= self.radius,self.x >= wScreen - self.radius,self.y >= hScreen - self.radius)):
            return True
        else:
            return False
    def pathAssign(self,path):

        [self.pathX,self.pathY] = path

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


        self.edge =  ((x-0,y-0),
                     (x+width+0,y-0),
                     (x-0,y+heigh+0),
                     (x+width+0, y+heigh+0))     

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

def cubic_spline_planer(x,y):
    ds = 0.1  # [m] distance of each interpolated points

    sp = Spline2D(x, y)
    s = np.arange(0, sp.s[-1], ds)

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = sp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(sp.calc_yaw(i_s))
        rk.append(sp.calc_curvature(i_s))
    
    return rx,ry,ryaw,rk


def main():
    #Robot initialize
    rb1 = robot(0,0,robotRadius,(255,255,255))
    rb1.x = 100
    rb1.y = 100
    listnodes = list()

    #Obstacles initialize
    otc = list()
    otc.append(obstacles(5,0,1,2,(255,0,0)))
    otc.append(obstacles(10,0,1,2,(255,0,0)))
    otc.append(obstacles(15,0,1,2,(255,0,0)))

    otc.append(obstacles(5,6,5,2,(152,152,152)))
    otc.append(obstacles(10,6,5,2,(152,152,152)))
    otc.append(obstacles(15,6,5,2,(152,152,152)))
    otc.append(obstacles(4,14,3,5,(152,152,152)))
    otc.append(obstacles(12,14,3,5,(152,152,152)))

    otcs_nodes = []
    for elm in otc:
        otcs_nodes.extend(elm.nodes)

    access_nodes = list()
    for row in range(1,numGridX):
        for col in range(1,numGridY):
            if (row,col) not in otcs_nodes:
                access_nodes.append((row,col))

    



    #Not safe area:
    othernodes = list()
    # for col in range(1,numGridX-1):
    #     for row in range(1,numGridY-1):
    #         if ( col%2 == 0 and row %2 ==0):
    #             centerX = col*int((wScreen/numGridX))
    #             centerY = row*int((hScreen/numGridY))
    #             othernodes.append( obstacles(centerX,centerY,0,0,(0,0,0)))
    #             # othernodes.append( DijkstraSearch.Node(centerX-robotRadius,centerY-robotRadius))
    #             # othernodes.append( DijkstraSearch.Node(centerX+robotRadius,centerY+robotRadius))
    #             # othernodes.append( DijkstraSearch.Node(centerX+robotRadius,centerY-robotRadius))
    #             # othernodes.append( DijkstraSearch.Node(centerX-robotRadius,centerY+robotRadius))




    #Path planning
    for elm in otc :
        for node in elm.nodes:
            listnodes.append(node)

    sx, sy = 1, 1  # [m]
    gx, gy = 16, 19  # [m]
    expand_distance = robotRadius

    rx, ry = VisibilityRoadMap(expand_distance, do_plot=False)\
        .planning(sx, sy, gx, gy, access_nodes)
    # Apply smooth path:
    # rx,ry,ryaw,rk = cubic_spline_planer(rx,ry)
    
    rb1.pathAssign(VisibilityRoadMap(expand_distance, do_plot=False)\
        .planning(sx, sy, gx, gy, access_nodes))
    

    
    print(rb1.pathY)
    print("----------")

    print(ry)
    # print(rx,ry)
    rb1.x = rb1.pathX[0]*sectorSize
    rb1.y = rb1.pathY[0]*sectorSize

    run = True


    #set initial position of Robot
    
    

    clock = pygame.time.Clock()
    while run:
        clock.tick(200)

        #
        screen.fill((255,255,255))
 
        rb1.draw(screen)
        for elm in otc :
            elm.draw(screen)



        
        for i in range(numGridX):
            pygame.draw.line(screen, (224,224,224),(i*int((wScreen/numGridX)),0), (i*int((wScreen/numGridX)),hScreen))

        for i in range(numGridY):
            pygame.draw.line(screen, (224,224,224),(0,i*int((hScreen/numGridY))), (wScreen , i*int((hScreen/numGridY))))


        # for elm in access_nodes:
        #    pygame.draw.circle(screen, (255,0,0),(elm[0]*sectorSize,elm[1]*sectorSize), 2)    
           
        for i in range(len(rx)-1):
            pygame.draw.line(screen,(255,0,0),(rx[i]*sectorSize,ry[i]*sectorSize),(rx[i+1]*sectorSize,ry[i+1]*sectorSize))

        pygame.display.update() 
        
        #Check if it hit the obstacle
        for elm in otc:
            if elm.getHit(rb1):
                orient = orient + math.pi/2
        if rb1.hitBoundaries():
            orient = orient + math.pi/2


        rb1.move(speedMax)    
        if(rb1.reachNodePath() and rb1.indexPath < len(rb1.pathX)-1):
            rb1.indexPath += 1

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False

            if event.type == pygame.MOUSEBUTTONDOWN:
                pos =pygame.mouse.get_pos()



    pygame.quit()
    quit()


if __name__ == '__main__':
    pygame.display.set_caption('MMRS')
    main()
