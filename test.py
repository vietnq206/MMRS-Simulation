import pygame
import math


#Map init

wScreen = 500
hScreen = 500
numGrid = 20
timeStep = 0.01

win = pygame.display.set_mode((wScreen,hScreen))

pygame.display.set_caption('MMRS')







class robot(object):
    def __init__(self,x,y,radius,color):
        self.x = x
        self.y = y
        self.speed = 0
        self.radius = radius
        self.color = color

    def setSpeed(self,speed):
        self.speed = speed

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





def redrawWindow():
    win.fill((255,255,255))
    for i in range(numGrid):
        pygame.draw.line(win, (224,224,224),(i*int((wScreen/numGrid)),0), (i*int((wScreen/numGrid)),hScreen))
        pygame.draw.line(win, (224,224,224),(0,i*int((hScreen/numGrid))), (wScreen , i*int((hScreen/numGrid))))

    rb1.draw(win)
    pygame.draw.line(win, (0,0,0),line[0], line[1])
    pygame.display.update()

def findAngle(pos):
    sX = rb1.x
    sY = rb1.y
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


rb1 = robot(300,494,5,(255,255,255))

run = True
time = 0
power = 0
angle = 0
x = 0
y = 0
rb1.x = 0
rb1.y = 0
shoot = False
clock = pygame.time.Clock()
while run:
    clock.tick(200)

    time += 0.05
    po = robot.ballPath(x, y, power, angle, time)
    # rb1.x = po[0] 
    # rb1.y = po[1]
    if ( rb1.x > 300):
        rb1.speed = -20
    rb1.x = rb1.speed*timeStep + rb1.x 
    rb1.y = rb1.speed*timeStep + rb1.y
    line = [(rb1.x, rb1.y), pygame.mouse.get_pos()]
    redrawWindow()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False

        if event.type == pygame.MOUSEBUTTONDOWN:
            # if not shoot:
            time = 0
            rb1.setSpeed(50)
            x = rb1.x
            y = rb1.y
            pos =pygame.mouse.get_pos()
            shoot = True
            power = math.sqrt((line[1][1]-line[0][1])**2 +(line[1][0]-line[0][1])**2)/8
            angle = findAngle(pos)



pygame.quit()
quit()
