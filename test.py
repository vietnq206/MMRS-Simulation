from turtle import width
import pygame
import math
import numpy as np


#Map init

wScreen = 500
hScreen = 500
numGrid = 20
timeStep = 0.01

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

    def draw(self,screen):
        pygame.draw.rect(screen, self.color, pygame.Rect(self.x, self.y, self.width, self.heigh))
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



    

def findAngle(pos,rb):
    sX = rb.x
    sY = rb.y
    try:
        angle = math.atan((sY - pos[1]) / (sX - pos[0]))
    except:
        angle = math.pi / 2

    if pos[1] < sY and pos[0] > sX:
        angle = abs(angle)
    elif pos[1] < sY and pos[0] < sX:
        angle = math.pi - angle
    elif pos[1] > sY and pos[0] < sX:
        angle = math.pi + abs(angle)
    elif pos[1] > sY and pos[0] > sX:
        angle = (math.pi * 2) - angle

    return angle

def main():
    #Robot initialize
    rb1 = robot(0,0,5,(255,255,255))
    rb1.x = 300
    rb1.y = 300


    #Obstacles initialize
    otc1 = obstacles(100,100,100,50,(152,152,152))

    run = True
    #set initial position of Robot
    screen.fill((255,255,255))
    for i in range(numGrid):
        pygame.draw.line(screen, (224,224,224),(i*int((wScreen/numGrid)),0), (i*int((wScreen/numGrid)),hScreen))
        pygame.draw.line(screen, (224,224,224),(0,i*int((hScreen/numGrid))), (wScreen , i*int((hScreen/numGrid))))

    clock = pygame.time.Clock()
    while run:
        clock.tick(200)

        


        rb1.draw(screen)
        otc1.draw(screen)
        
        pygame.display.update() 

        
        

        if otc1.getHit(rb1):
            rb1.move(0,0)
        else:
            rb1.move( 3*math.pi/4, 50)

            
            

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False

            if event.type == pygame.MOUSEBUTTONDOWN:
                pos =pygame.mouse.get_pos()



    pygame.quit()
    quit()

main()
