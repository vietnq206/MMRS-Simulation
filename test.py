# from pathPlaning import *
 
# import numpy as np
# import json
# robotRadius = 6
# numGridX = 20
# numGridY = 20
 

# data = json.loads('task_list.json')


# # print( finalNumpyArrayOne)

 


# otc = list()
# otc.append(obstacles(5,0,1,2,(255,0,0)))
# otc.append(obstacles(10,0,1,2,(255,0,0)))
# otc.append(obstacles(15,0,1,2,(255,0,0)))

# otc.append(obstacles(5,6,5,2,(152,152,152)))
# otc.append(obstacles(10,6,5,2,(152,152,152)))
# otc.append(obstacles(15,6,5,2,(152,152,152)))
# otc.append(obstacles(4,14,3,5,(152,152,152)))
# otc.append(obstacles(12,14,3,5,(152,152,152)))

# otcs_nodes = []
# for elm in otc:
#     otcs_nodes.extend(elm.nodes)

# access_nodes = list()
# for row in range(1,numGridX):
#     for col in range(1,numGridY):
#         if (row,col) not in otcs_nodes:
#             access_nodes.append((row,col))



# sx, sy = 3, 5  # [m]
# gx, gy = 1, 3  # [m]
# expand_distance = robotRadius

# rx, ry = VisibilityRoadMap(expand_distance, do_plot=False)\
#     .planning(sx, sy, gx, gy, access_nodes)

# print(rx)
# print("----------")
# print(ry)
# def direction(p1,p2):
#     if p1[0] == p2 [0]:
#         if p2[1] > p1[1]:
#             return 4
#         else:
#             return 0
#     if p2[1] == p1[1]:
#         if p2[0] > p1[1]:
#             return 6
#         else:
#             return 2
#     if p2[0] > p1[0]:
#         if p2[1] > p1[1]:
#             return 5
#         else:
#             return 7

#     if p2[0] < p1[0]:
#         if p2[1] > p1[1]:
#             return 3
#         else:
#             return 1


# print(direction([3,3],[4,3]))




