from Supervisor import *



import json
from json import JSONEncoder
class NumpyArrayEncoder(JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return JSONEncoder.default(self, obj)
 
def main():

    screen= pygame.display.set_mode((wScreen,hScreen))
    #Robot initiali
    # ze
    robots = list()
    robots.append(robot(0,0,robotRadius,20,(0,0,255)))
    robots.append(robot(0,0,robotRadius,20,(0,0,255)))
    robots.append(robot(0,0,robotRadius,20,(0,0,255)))
    robots.append(robot(0,0,robotRadius,20,(0,0,255)))
 
    robots.append(robot(0,0,robotRadius,20,(0,0,255)))
    robots.append(robot(0,0,robotRadius,20,(0,0,255)))
    robots.append(robot(0,0,robotRadius,20,(0,0,255)))
    robots.append(robot(0,0,robotRadius,20,(0,0,255)))
    robots.append(robot(0,0,robotRadius,20,(0,0,255)))
    robots.append(robot(0,0,robotRadius,20,(0,0,255)))
 

    robots.append(robot(0,0,robotRadius,20,(0,0,255)))
    robots.append(robot(0,0,robotRadius,20,(0,0,255)))
    robots.append(robot(0,0,robotRadius,20,(0,0,255)))
    robots.append(robot(0,0,robotRadius,20,(0,0,255)))
    robots.append(robot(0,0,robotRadius,20,(0,0,255)))

 

    # robots.append(robot(0,0,robotRadius,20,(0,0,255)))
    # robots.append(robot(0,0,robotRadius,20,(0,0,255)))
    # robots.append(robot(0,0,robotRadius,20,(0,0,255)))
    # robots.append(robot(0,0,robotRadius,20,(0,0,255)))
    # robots.append(robot(0,0,robotRadius,20,(0,0,255)))


    # robots.append(robot(0,0,robotRadius,25,(223,124,216)))
    # robots.append(robot(0,0,robotRadius,25,(234,124,221)))
    # robots.append(robot(0,0,robotRadius,25,(246,224,121)))
    # robots.append(robot(0,0,robotRadius,25,(215,124,134)))
    # robots.append(robot(0,0,robotRadius,25,(255,124,189)))






    #Obstacles initialize
    otc = list()

    #Outport
    otc.append(obstacles(5,0,1,2,(255,0,0)))
    otc.append(obstacles(10,0,1,2,(255,0,0)))
    otc.append(obstacles(15,0,1,2,(255,0,0)))
    #Start place
    # otc.append(obstacles(0,11,0,14,(255,255,0)))
    # otc.append(obstacles(19,11,0,14,(255,255,0)))


    #package shef
    otc.append(obstacles(4,6,5,2,(152,152,152)))
    otc.append(obstacles(9,6,5,2,(152,152,152)))
    otc.append(obstacles(14,6,5,2,(152,152,152)))
    otc.append(obstacles(4,14,3,5,(152,152,152)))
    otc.append(obstacles(12,14,3,5,(152,152,152)))

    otcs_nodes = []
    for elm in otc:
        otcs_nodes.extend(elm.nodes)

    access_nodes = list()
    for row in range(0,numGridX):
        for col in range(0,numGridY):
            if (row,col) not in otcs_nodes:
                access_nodes.append((row,col))

    for row in range(0,numGridX-1):
        for col in range(0,numGridY-1):
            if (row+0.5,col+0.5) not in otcs_nodes:
                access_nodes.append((row+0.5,col+0.5))



    for i in range(8):
        robots[i].loc_node_x = 1
        robots[i].loc_node_y = 1+i*2
        # robots[i].pathAssign(VisibilityRoadMap(robotRadius, do_plot=False)\
        #     .planning(1 ,1+i*2, 19, 19-i*2, access_nodes)) 

    for i in range(8,15):
        robots[i].loc_node_x = 19
        robots[i].loc_node_y = 2+(i-8)*2
    # robots[12].spotlight = True
    #     # robots[i].pathAssign(VisibilityRoadMap(robotRadius, do_plot=False)\
    #     # .planning(19 ,2+(i-8)*2, 1, 18-(i-8)*2, access_nodes)) 

    # for i in range(15,20):
    #     robots[i].loc_node_x = 4+(i-15)*2
    #     robots[i].loc_node_y = 3

        # robots[i].pathAssign(VisibilityRoadMap(robotRadius, do_plot=False)\
        # .planning(4+(i-15)*2,3, 6+(i-15) , 19, access_nodes)) 

    # for i in range(15):
    #     print("Robot: "+str(i))
    #     print(robots[i].path)
    #     print("------------")
    # #Path planning
    # robots[0].pathAssign(VisibilityRoadMap(robotRadius, do_plot=False)\
    #     .planning(16 ,19, 1, 1, access_nodes))
    # robots[1].pathAssign(VisibilityRoadMap(robotRadius, do_plot=False)\
    #     .planning(1, 19, 18, 3, access_nodes))
    # robots[2].pathAssign(VisibilityRoadMap(robotRadius, do_plot=False)\
    #     .planning(1, 4, 18, 6, access_nodes))
    # robots[3].pathAssign(VisibilityRoadMap(robotRadius, do_plot=False)\
    #     .planning(18, 3, 4, 18, access_nodes))
    # robots[4].pathAssign(VisibilityRoadMap(robotRadius, do_plot=False)\
    #     .planning(2,6, 14, 10, access_nodes))    


    # loc_package_load = [[3,7],[3,10],[7,7],[7,10],[8,7],[8,10],[12,7],[12,10],[13,7],[13,10],[17,7],[17,10],[5,13],[8,13],[5,18],[8,18],[13,13],[16,13],[13,18],[16,18],[6,2],[11,2],[16,2]]
    # array_load = np.zeros((len(loc_package_load),len(loc_package_load)))
    # for i in range(len(loc_package_load)):
    #     print("Doing for loc: "+ str(i))
    #     for j in range(len(loc_package_load)):
    #         print("      Apply for loc: "+ str(j))

    #         if i !=j:
    #             rx,ry = VisibilityRoadMap(robotRadius, do_plot=False).planning(loc_package_load[i][0],loc_package_load[i][1],\
    #                                                                            loc_package_load[j][0],loc_package_load[j][1], access_nodes)
    #             for k in range(len(rx)-1):
    #                 array_load[i][j] += np.sqrt((rx[k+1]-rx[k])**2 + (ry[k+1]-ry[k])**2)
                 
    # numpyData = {"arrayOne": array_load}               

    # with open("numpyData.json", "w") as write_file:
    #     json.dump(numpyData, write_file, cls=NumpyArrayEncoder)



 
    run = True

    #Superviosr initialize
    supervisor = Supervisor(robots,access_nodes)
    supervisor.print_register_map()

    print("Import tasl:")
    supervisor.import_task( )
    print("----------")

    





    clock = pygame.time.Clock()
    c = 1
    nhap = 1
    runTime = 0
    while run:
        runTime += 1
        print(runTime)
        clock.tick(200)

        supervisor.check_collision()
        supervisor.release_register()
        
        # supervisor.print_register_map()
        
        #

        #Scenario display
        screen.fill((255,255,255))
        for i in range(numGridX):
            pygame.draw.line(screen, (224,224,224),(i*int((wScreen/numGridX)),0), (i*int((wScreen/numGridX)),hScreen))
            pygame.draw.line(screen, (224,224,224),(i*int((wScreen/numGridX)),0), (wScreen , hScreen-i*int((hScreen/numGridY))))
            pygame.draw.line(screen, (224,224,224),(i*int((wScreen/numGridX)),0), (0 ,  i*int((hScreen/numGridY))))
            pygame.draw.line(screen, (224,224,224),(i*int((wScreen/numGridX)),hScreen), (wScreen ,  i*int((hScreen/numGridY))))
        for i in range(numGridY):
            pygame.draw.line(screen, (224,224,224),(0,i*int((hScreen/numGridY))), (wScreen , i*int((hScreen/numGridY))))
            pygame.draw.line(screen, (224,224,224),(0,i*int((hScreen/numGridY))), (wScreen-i*int((wScreen/numGridX)),hScreen))
            pygame.draw.line(screen, (224,224,224),(0,i*int((hScreen/numGridY))), ( i*int((wScreen/numGridX)),0))





        for elm in otc :
            elm.draw(screen)
        # print("--------d--")
        #Task assignment:
        if nhap == 1:
            for rb in range(len(robots)):
            
            # if robots[rb].get_state() in [st_DONE,st_STOP]:
                


                path = supervisor.gen_Path(rb,0)
                # print("Path generate of robot: " + str(rb))
                # print(path)
                print("----------")
                robots[rb].path = path
                robots[rb].x = path[0][0]*sectorSize
                robots[rb].y = path[0][1]*sectorSize              

            input() 
            nhap = 0
        # print("--------dx--")
        supervisor.ask_register()
        #Robots processes
        for rb in range(len(robots)):
            # print("Robot: " + str(rb))
            # print("location path:" + str(robots[rb].x/sectorSize) +"and loc y: "+str(robots[rb].y/sectorSize))
            robots[rb].draw(screen)
            # if rb == 12:
            #     for i in range(len(robots[rb].path)-1):
            #         pygame.draw.line(screen,(255,0,0),robots[rb].path[i]*sectorSize,robots[rb].path[i+1]*sectorSize)

            robots[rb].reachNodePath()
            if( robots[rb].indexPath < len(robots[rb].path)):
                robots[rb].move()
            else:
                robots[rb].speed = 0
                robots[rb].move()
        
        pygame.display.update() 


        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False

            if event.type == pygame.MOUSEBUTTONDOWN:
                supervisor.print_register_map()
                pos =pygame.mouse.get_pos()



    pygame.quit()
    quit()


if __name__ == '__main__':
    pygame.display.set_caption('MMRS')
    main()
