from turtle import width
import pygame
import math
import numpy as np
from pathPlaning import *

#Map init

wScreen = 800
hScreen = 800
numGridX = 20
numGridY = 20
sectorSize = int(wScreen/20)


timeStep = 0.01
robotRadius = 6

screen= pygame.display.set_mode((wScreen,hScreen))

pygame.display.set_caption('MMRS')








class robot(object):
    def __init__(self,x,y,radius,color):
        self.x = x
        self.y = y
        self.speed = 0
        self.orient = 0
        self.radius = radius
        self.color = color
        


    def move(self,orient,speed):
        self.orient = orient
        self.speed = speed
        velx = math.cos(self.orient) * self.speed
        vely = - math.sin(self.orient) * self.speed
        self.x = velx*timeStep + self.x 
        self.y = vely*timeStep + self.y

    def draw(self, win):
        pygame.draw.circle(win, (0,0,0), (self.x,self.y), self.radius)
        pygame.draw.circle(win, self.color, (self.x,self.y), self.radius-1)

    def hitBoundaries(self):
        if any( (self.x <= self.radius,self.y <= self.radius,self.x >= wScreen - self.radius,self.y >= hScreen - self.radius)):
            return True
        else:
            return False

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
        self.x = x
        self.y = y
        self.heigh = heigh
        self.width = width
        self.color = color
        self.nodes = ((x-robotRadius,y-robotRadius),
                     (x+width+robotRadius,y-robotRadius),
                     (x-robotRadius,y+heigh+robotRadius),
                     (x+width+robotRadius, y+heigh+robotRadius))   
        self.edge =  ((x-0,y-0),
                     (x+width+0,y-0),
                     (x-0,y+heigh+0),
                     (x+width+0, y+heigh+0))     

    def draw(self,screen):
        pygame.draw.rect(screen, self.color, pygame.Rect(self.x, self.y, self.width, self.heigh))
        for elm in self.nodes:
            pygame.draw.circle(screen, (255,0,0), elm, 2)
    def getHit(self,rb):
        if (
            ( (np.abs(rb.x-self.x)<=rb.radius or np.abs(rb.x-self.x - self.width)<=rb.radius)  and rb.y > self.y and rb.y < self.y + self.heigh)  or \
            ( (np.abs(rb.y-self.y)<=rb.radius or np.abs(rb.y-self.y - self.heigh)<=rb.radius ) and rb.x > self.x and rb.x < self.x + self.width) or  \
            (math.sqrt((rb.x-self.x)**2 + (rb.y  - self.y)**2) < rb.radius ) or \
            (math.sqrt((rb.x-self.x-self.width)**2 + (rb.y  - self.y - self.heigh)**2) < rb.radius ) or \
            (math.sqrt((rb.x-self.x)**2 + (rb.y  - self.y - self.heigh)**2) < rb.radius ) or \
            (math.sqrt((rb.x-self.x-self.width)**2 + (rb.y  - self.y)**2) < rb.radius ) ):
            
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
    rb1.x = 150
    rb1.y = 75
    listnodes = list()

    #Obstacles initialize
    otc = list()
    otc.append(obstacles(sectorSize*5,0,sectorSize,sectorSize*2,(255,0,0)))
    otc.append(obstacles(sectorSize*10,0,sectorSize,sectorSize*2,(255,0,0)))
    otc.append(obstacles(sectorSize*15,0,sectorSize,sectorSize*2,(255,0,0)))

    otc.append(obstacles(sectorSize*5,sectorSize*6,sectorSize*5,sectorSize*2,(152,152,152)))
    otc.append(obstacles(sectorSize*10,sectorSize*6,sectorSize*5,sectorSize*2,(152,152,152)))
    otc.append(obstacles(sectorSize*15,sectorSize*6,sectorSize*5,sectorSize*2,(152,152,152)))
    otc.append(obstacles(sectorSize*4,sectorSize*14,sectorSize*3,sectorSize*5,(152,152,152)))
    otc.append(obstacles(sectorSize*12,sectorSize*14,sectorSize*3,sectorSize*5,(152,152,152)))



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

    sx, sy = 0, 200  # [m]
    gx, gy = 500, 425  # [m]
    expand_distance = robotRadius

    rx, ry = VisibilityRoadMap(expand_distance, do_plot=False)\
        .planning(sx, sy, gx, gy, otc, othernodes)
    # Apply smooth path:
    # rx,ry,ryaw,rk = cubic_spline_planer(rx,ry)



    # print(rx,ry)
    run = True

    #set initial position of Robot
    orient = -math.pi/4
 
    clock = pygame.time.Clock()
    while run:
        clock.tick(200)

        #
        screen.fill((255,255,255))
 
        rb1.draw(screen)
        for elm in otc :
            elm.draw(screen)

        for i in range(len(rx)-1):
            pygame.draw.line(screen,(255,0,0),(rx[i],ry[i]),(rx[i+1],ry[i+1]))

        
        for i in range(numGridX):
            pygame.draw.line(screen, (224,224,224),(i*int((wScreen/numGridX)),0), (i*int((wScreen/numGridX)),hScreen))

        for i in range(numGridY):
            pygame.draw.line(screen, (224,224,224),(0,i*int((hScreen/numGridY))), (wScreen , i*int((hScreen/numGridY))))





        pygame.display.update() 
        
        #Check if it hit the obstacle
        for elm in otc:
            if elm.getHit(rb1):
                orient = orient + math.pi/2
        if rb1.hitBoundaries():
            orient = orient + math.pi/2


        rb1.move( orient, 50)    

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False

            if event.type == pygame.MOUSEBUTTONDOWN:
                pos =pygame.mouse.get_pos()



    pygame.quit()
    quit()

main()
