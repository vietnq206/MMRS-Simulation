
import numpy as np
wScreen = 800
hScreen = 800
numGridX = 20
numGridY = 20
sectorSize = int(wScreen/20)

speedMax = 20
timeStep = 0.01
robotRadius = 6
##############
st_STOP = 0
st_RUN = 1
st_ASK = 2




class Supervisor:
    def __init__(self,robots):
        self.robots = robots
        self.MapToken = np.zeros( (numGridX,numGridY) , dtype=np.int64) - 1

        for idx in range(len(robots)):
            if self.MapToken[robots[idx].path[0][0]][robots[idx].path[0][1]] == -1:
                self.MapToken[robots[idx].path[0][0]][robots[idx].path[0][1]] = idx
    
    def release_register(self):
        for idx in range(len(self.robots)):
            if(self.robots[idx].release_prevNode):
               self.MapToken[self.robots[idx].release_node()[0]][self.robots[idx].release_node()[1]] = -1       
    def ask_register(self):
        for idx in range(len(self.robots)):
         
            if self.robots[idx].state == st_ASK:
                if self.MapToken[self.robots[idx].asking_node()[0]][self.robots[idx].asking_node()[1]] == -1:
                    self.MapToken[self.robots[idx].asking_node()[0]][self.robots[idx].asking_node()[1]] = idx
                    self.robots[idx].request_accepted()
            


    def print_register_map(self):
        print("Map all:")
        for i in range(numGridX):
            s = ""
            for j in range(numGridY):
                s = s + str(self.MapToken[i][j]) +" "
            print(s)

    
