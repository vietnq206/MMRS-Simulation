from Supervisor import *



 
def main():

    screen= pygame.display.set_mode((wScreen,hScreen))
    #Robot initialize
    robots = list()
    robots.append(robot(0,0,robotRadius,(255,255,222)))
    robots.append(robot(0,0,robotRadius,(255,111,222)))
    robots.append(robot(0,0,robotRadius,(143,111,124)))
    robots.append(robot(0,0,robotRadius,(123,235,222)))
    robots.append(robot(0,0,robotRadius,(255,211,104)))



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
    for row in range(0,numGridX):
        for col in range(0,numGridY):
            if (row,col) not in otcs_nodes:
                access_nodes.append((row,col))

    


    #Path planning
    robots[0].pathAssign(VisibilityRoadMap(robotRadius, do_plot=False)\
        .planning(16 ,19, 1, 1, access_nodes))
    robots[1].pathAssign(VisibilityRoadMap(robotRadius, do_plot=False)\
        .planning(1, 19, 18, 3, access_nodes))
    robots[2].pathAssign(VisibilityRoadMap(robotRadius, do_plot=False)\
        .planning(1, 4, 18, 6, access_nodes))
    robots[3].pathAssign(VisibilityRoadMap(robotRadius, do_plot=False)\
        .planning(18, 3, 4, 18, access_nodes))
    robots[4].pathAssign(VisibilityRoadMap(robotRadius, do_plot=False)\
        .planning(2,6, 14, 10, access_nodes))    

    
   
    print("----------")

 
    run = True

    #Superviosr initialize
    supervisor = Supervisor(robots)
    supervisor.print_register_map()


    clock = pygame.time.Clock()
    while run:
        clock.tick(200)
        supervisor.ask_register()
        # supervisor.print_register_map()
        supervisor.release_register()
        #

        #Scenario display
        screen.fill((255,255,255))
        for i in range(numGridX):
            pygame.draw.line(screen, (224,224,224),(i*int((wScreen/numGridX)),0), (i*int((wScreen/numGridX)),hScreen))
        for i in range(numGridY):
            pygame.draw.line(screen, (224,224,224),(0,i*int((hScreen/numGridY))), (wScreen , i*int((hScreen/numGridY))))
        for elm in otc :
            elm.draw(screen)


        #Robots processes
        for rb in robots:
            rb.draw(screen)
            for i in range(len(rb.path)-1):
                pygame.draw.line(screen,(255,0,0),rb.path[i]*sectorSize,rb.path[i+1]*sectorSize)

            rb.reachNodePath()
            if( rb.indexPath < len(rb.path)):
                rb.move(speedMax)
            else:
                rb.move(0)

        pygame.display.update() 
        

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
