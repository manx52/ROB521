# ======
# ROB521_Assignment1.py
# ======
#
# This assignment will introduce you to the idea of motion planning for  
# holonomic robots that can move in any direction and change direction of 
# motion instantaneously.  Although unrealistic, it can work quite well for
# complex large scale planning.  You will generate mazes to plan through 
# and employ the PRM algorithm presented in lecture as well as any 
# variations you can invent in the later sections.
# 
# There are three questions to complete (5 marks each):
#
#    Question 1: implement the PRM algorithm to construct a graph
#    connecting start to finish nodes.
#    Question 2: find the shortest path over the graph by implementing the
#    Dijkstra's or A* algorithm.
#    Question 3: identify sampling, connection or collision checking 
#    strategies that can reduce runtime for mazes.
#
# Fill in the required sections of this script with your code, run it to
# generate the requested plots, then paste the plots into a short report
# that includes a few comments about what you've observed.  Append your
# version of this script to the report.  Hand in the report as a PDF file.
#
# requires: basic python,
#
# S L Waslander, January 2022
import time
import sys
import numpy as np
import random
import matplotlib.pyplot as plt
import maze
import show_maze
import MinDist2Edges
import CheckCollision


# set random seed for repeatability if desired
np.random.seed(1337)

# ==========================
# Maze Generation
# ==========================
#
# The maze function returns a map object with all of the edges in the maze.
# Each row of the map structure draws a single line of the maze.  The
# function returns the lines with coordinates [x1 y1 x2 y2].
# Bottom left corner of maze is [0.5 0.5],
# Top right corner is [col+0.5 row+0.5]
#
#
# row = 5 # Maze rows
# col = 7 # Maze columns
# maze_map = maze.maze(row,col) # Creates the maze
# start = [0.5, 1.0] # Start at the bottom left
# finish = [col+0.5, row] # Finish at the top right
#
#
# h = plt.figure(1)
# plt.clf()
# plt.plot(start[0], start[1], 'go')
# plt.plot(finish[0], finish[1], 'rx')
# show_maze.show_maze(maze_map,row,col,h) # Draws the maze
# plt.show()
#
# # ======================================================
# # Question 1: construct a PRM connecting start and finish
# # ======================================================
# #
# # Using 500 samples, construct a PRM graph whose milestones stay at least
# # 0.1 units away from all walls, using the MinDist2Edges function provided for
# # collision detection.  Use a nearest neighbour connection strategy and the
# # CheckCollision function provided for collision checking, and find an
# # appropriate number of connections to ensure a connection from  start to
# # finish with high probability.
#
#
# # variables to store PRM components
# nS = 500  # number of samples to try for milestone creation
# milestones = [start, finish]  # each row is a point [x y] in feasible space
# edges = []  # each row is should be an edge of the form [x1 y1 x2 y2]
#
# print("Time to create PRM graph")
# t = time.time()
# # ------insert your PRM generation code here-------
#
# # 1. Generate N random points in maze boundary
# x_high = col+0.5
# x_low = 0.5
# y_high = row+0.5
# y_low = 0.5
# x = (x_high-x_low) * np.random.random((nS,1)) + x_low
# y = (y_high-y_low) * np.random.random((nS,1) )+ y_low
# pts = np.array([x, y]).T[0]
#
#
# # 2.remove collided points if its 0.1 nearest to a wall
# dist = MinDist2Edges.MinDist2Edges(pts,maze_map)
# indices = []
# for i in range(nS):
#      if dist[i] < 0.1:
#          indices.append(i)
#
# pts = np.delete(pts, indices, axis=0)
#
# milestones = np.concatenate((milestones, pts), axis=0)
#
# # 3. Find nearest 10 neighbors and link them
#
# for i in range(len(milestones)):
#     x_d = milestones[i,0] - milestones[:,0]
#     y_d = milestones[i,1] - milestones[:,1]
#     distance = np.sqrt(x_d**2 + y_d**2)
#     sorted_indices = np.argsort(distance)
#
#     for j in range(12):
#         [inCollision, edge] = CheckCollision.CheckCollision(milestones[i,:], milestones[sorted_indices[j],:], maze_map)
#         if inCollision == 0: # check for collision in the edge path
#             edges.append(np.concatenate((milestones[i,:], milestones[sorted_indices[j],:]), axis=0))
#
# edges = np.array(edges)
#
# # ------end of your PRM generation code -------
#
# elapsed = time.time() - t
# print("Elapsed time is ",elapsed, " seconds.")
#
# h = plt.figure(1)
# plt.clf()
# plt.plot(start[0], start[1], 'go')
# plt.plot(finish[0], finish[1], 'rx')
# plt.plot(milestones[:,0], milestones[:,1], 'm.')
# if len(edges) != 0:
#     plt.plot(edges[:, [0,2]].T, edges[:, [1,3]].T, color='magenta')
# plt.title(f'Q1 - {row} X {col} Maze PRM')
# show_maze.show_maze(maze_map,row,col,h) # Draws the maze
# plt.savefig("assignment1_q1.png")
# plt.show()
#
# # =================================================================
# # Question 2: Find the shortest path over the PRM graph
# # =================================================================
# #
# # Using an optimal graph search method (Dijkstra's or A*) , find the
# # shortest path across the graph generated.  Please code your own
# # implementation instead of using any built in functions.
#
# print('Time to find shortest path')
# t = time.time()
#
# # Variable to store shortest path
# spath = [] # shortest path, stored as a milestone row index sequence
#
#
# # ------insert your shortest path finding algorithm here-------
# def find(lst):
#     return [i for i, x in enumerate(lst) if x]
#
# unvisted = np.arange(1,len(milestones),1)  # removed start
#
# cost_gx = np.inf*np.ones((len(milestones),1))
#
# # A*
#
# # Calculate h(x) for start
# x_d = start[0] - finish[0]
# y_d = start[1] - finish[1]
# hx = np.sqrt(y_d**2 + x_d**2)
# cost = 0 + hx
#
# # Tracks previous obtained milestone
# previous = -1*np.ones((len(milestones),1))#np.full((len(milestones), 1), -1)
#
# # Add start to Q
# Q_idx = [0]
# Q_cost = [cost]
# cost_gx[0] = 0
#
# success = False
#
# # A*
# while len(Q_idx) != 0:
#     # select first from list
#     x_idx = Q_idx[0]
#     x_cost = Q_cost[0]
#     # Delete from list
#     Q_idx = Q_idx[1:]
#     Q_cost = Q_cost[1:]
#     # Found goal
#     if x_idx == 1:
#         success = True
#         break
#
#
#     # Get edges
#     from_edge_idx = np.where(np.isin(edges[:, :2], milestones[x_idx, :]))[0]
#
#     # find milestone from edges
#     pt_idx = np.where(np.isin(milestones[:, :2], edges[from_edge_idx, 2:4]))[0]
#
#     # Go through all neighbours
#     for i in range(len(pt_idx)):
#         # Check if points are visited or not
#         if pt_idx[i] in unvisted:
#             # remove from list
#             unvisted = np.delete(unvisted, np.where(unvisted == pt_idx[i]))
#
#             # Calculate g(x) and c(x) between edges
#             x_d = milestones[pt_idx[i], 0] - milestones[x_idx, 0]
#             y_d = milestones[pt_idx[i], 1] - milestones[x_idx, 1]
#             cx = np.sqrt(x_d**2 + y_d**2)
#             gx = x_cost + cx
#
#             # only select paths that are more efficient than previous
#             if gx < cost_gx[pt_idx[i]]:
#                 cost_gx[pt_idx[i]] = gx
#                 previous[pt_idx[i]] = x_idx
#
#                 # Calculate cost and h(x)
#                 x_d = milestones[pt_idx[i], 0] - finish[0]
#                 y_d = milestones[pt_idx[i], 1] - finish[1]
#                 hx = abs(y_d) + abs(x_d)
#                 cost = gx + hx
#
#                 # Adding to Q depends on rank
#                 if not Q_cost:
#                     Q_idx.append(pt_idx[i])
#                     Q_cost.append(cost)
#                 else:
#                     id = find(Q_cost < cost)
#
#                     if not id:  # smallest value
#                         Q_cost = [cost] + Q_cost
#                         Q_idx = [pt_idx[i]] + Q_idx
#                     else:  # In the middle
#                         id = id[-1]
#                         Q_cost = Q_cost[:id+1] + [cost] + Q_cost[id+1:]
#                         Q_idx = Q_idx[:id+1] + [pt_idx[i]] + Q_idx[id+1:]
#
# # create the shortest path
# if success:
#     idx = 1
#     spath = [1]
#     while idx != 0:
#         spath = [int(previous[idx][0])] + spath
#         idx = int(previous[idx][0])
# else:
#     print("No path exists")
#
# # ------end of shortest path finding algorithm-------
#
# elapsed = time.time() - t
# print("Elapsed time is ",elapsed, " seconds.")
#
# # plot the shortest path
# h = plt.figure(1)
# plt.clf()
# plt.plot(start[0], start[1], 'go')
# plt.plot(finish[0], finish[1], 'rx')
# plt.plot(milestones[:,0], milestones[:,1], 'm.')
# if len(edges) != 0:
#     plt.plot(edges[:, [0,2]].T, edges[:, [1,3]].T, color='magenta')
# for i in range(len(spath)):
#     plt.plot(milestones[spath[i:i+2], 0], milestones[spath[i:i+2], 1],  'go-', linewidth=3)
# str = "Q2 - {} X {} Maze Shortest Path".format(row, col)
# plt.title(str)
# show_maze.show_maze(maze_map,row,col,h) # Draws the maze
# plt.savefig("assignment1_q2.png")
# plt.show()

# ================================================================
# Question 3: find a faster way
# ================================================================
#
# Modify your milestone generation, edge connection, collision detection
# and/or shortest path methods to reduce runtime.  What is the largest maze
# for which you can find a shortest path from start to goal in under 20
# seconds on your computer? (Anything larger than 40x40 will suffice for
# full marks)

row = 5 # Maze rows
col = 5 # Maze columns
maze_map = maze.maze(row,col) # Creates the maze
start = [0.5, 1.0] # Start at the bottom left
finish = [col+0.5, row] # Finish at the top right
milestones = [start, finish]  # each row is a point [x y] in feasible space
edges = []  # each row is should be an edge of the form [x1 y1 x2 y2]

h = plt.figure(1)
plt.clf()
plt.plot(start[0], start[1], 'go')
plt.plot(finish[0], finish[1], 'rx')
show_maze.show_maze(maze_map,row,col,h) # Draws the maze
plt.show()

print("Attempting large {} X {} maze... \n".format(row,col))
t1 = time.time()


# ------insert your optimized algorithm here------

# ------insert your PRM generation code here-------
# 1. Generate N points uniformally in maze boundary
x_high = col+0.5
x_low = 0.5
y_high = row+0.5
y_low = 0.5
pts = []
x_scale = x_high - x_low
y_scale = y_high - y_low
for i in np.linspace(0.51,col ,int(col/0.5)): #range(1,col+1,0.1):
    for j in np.linspace(0.51,row,int(row/0.5)): #range(1,row+1,0.1):
        pts.append([i, j])

nS = len(pts)
pts = np.array(pts)
# 1. Generate N random points in maze boundary
# nS = 250  # number of samples to try for milestone creation

# x = (x_high-x_low) * np.random.random((nS,1)) + x_low
# y = (y_high-y_low) * np.random.random((nS,1) )+ y_low
# pts = np.array([x, y]).T[0]


# 2.remove collided points if its 0.1 nearest to a wall
dist = MinDist2Edges.MinDist2Edges(pts,maze_map)
indices = []
for i in range(nS):
     if dist[i] < 0.1:
         indices.append(i)

pts = np.delete(pts, indices, axis=0)

milestones = np.concatenate((milestones, pts), axis=0)

# 3. Find nearest 10 neighbors and link them

for i in range(len(milestones)):
    x_d = milestones[i,0] - milestones[:,0]
    y_d = milestones[i,1] - milestones[:,1]
    distance = np.sqrt(x_d**2 + y_d**2)
    sorted_indices = np.argsort(distance)

    for j in range(10):
        [inCollision, edge] = CheckCollision.CheckCollision(milestones[i,:], milestones[sorted_indices[j],:], maze_map)
        if inCollision == 0: # check for collision in the edge path
            edges.append(np.concatenate((milestones[i,:], milestones[sorted_indices[j],:]), axis=0))

edges = np.array(edges).reshape((len(edges),4))
# ------end of your PRM generation code -------

elapsed = time.time() - t1
print("PRM graph generation elapsed time is ",elapsed, " seconds.")

print('Time to find shortest path')
t2 = time.time()

# Variable to store shortest path
spath = [] # shortest path, stored as a milestone row index sequence


# ------insert your shortest path finding algorithm here-------
def find(lst):
    return [i for i, x in enumerate(lst) if x]

unvisted = np.arange(1,len(milestones),1)  # removed start

cost_gx = float("inf")*np.ones((len(milestones),1))

# A*

# Calculate h(x) for start
x_d = start[0] - finish[0]
y_d = start[1] - finish[1]
hx = np.sqrt(y_d**2 + x_d**2)
cost = 0 + hx

# Tracks previous obtained milestone
previous = -1*np.ones((len(milestones),1))#np.full((len(milestones), 1), -1)

# Add start to Q
Q_idx = [0]
Q_cost = [cost]
cost_gx[0] = 0

success = False
path = []
# A*
while len(Q_idx) != 0:
    # select first from list
    x_idx = Q_idx[0]
    x_cost = Q_cost[0]
    # Delete from list
    Q_idx = Q_idx[1:]
    Q_cost = Q_cost[1:]
    # print(x_idx)
    path.append(x_idx)

    # h = plt.figure(1)
    # plt.clf()
    # plt.plot(start[0], start[1], 'go')
    # plt.plot(finish[0], finish[1], 'rx')
    # plt.plot(milestones[:, 0], milestones[:, 1], 'm.')
    # if len(edges) != 0:
    #     plt.plot(edges[:, [0, 2]].T, edges[:, [1, 3]].T, color='magenta')
    # for i in range(len(path)):
    #     plt.plot(milestones[path[i:i + 1], 0], milestones[path[i:i + 1], 1], 'go-', linewidth=3)
    # for i in range(len(spath)):
    #     plt.plot(milestones[spath[i:i + 1], 0], milestones[spath[i:i + 1], 1], 'ro-', linewidth=3)
    # str = "Q3 - {} X {} Maze solved in {} seconds".format(row, col, elapsed)
    # plt.title(str)
    # show_maze.show_maze(maze_map, row, col, h)  # Draws the maze
    # # plt.savefig("assignment1_q3.png")
    # plt.show()

    # if np.in1d(x_idx, unvisted)[0]== 1:
    #     continue
    # Found goal
    if x_idx == 1:
        success = True
        break
    # unvisted = np.delete(unvisted, np.where(unvisted == x_idx))
    # Get edges

    # Get edges
    from_edge_idx = np.where(np.isin(edges[:, :2], milestones[x_idx, :]))[0]

    # find milestone from edges
    #pt_idx = np.where(np.isin(milestones[:, :2], edges[from_edge_idx, 2:4]))[0]
    # from_edge_idx = np.isin(edges[:, :2], milestones[0, :]).all(axis=1).ravel().nonzero()[0]
    # from_edge_idx2 = np.isin(edges[:, 2:4], milestones[2, :]).all(axis=1).ravel().nonzero()[0]
    # from_edge_idx = np.concatenate((from_edge_idx, from_edge_idx2))
    # # from_edge_idx = np.where(np.isin(edges[:, :2], milestones[x_idx,:]).all(axis=1))[0]
    pt_idx = np.isin(milestones[:,:2], edges[from_edge_idx, 2:4]).all(axis=1).ravel().nonzero()[0]
    # pt_idx2 = np.isin(milestones[:, :2], edges[pt_idx, 0:2]).all(axis=1).ravel().nonzero()[0]
    # pt_idx = np.concatenate((pt_idx, pt_idx2))
    #pt_idx =  np.where(np.isin(milestones[:, :2], edges[from_edge_idx, 2:4]).all(axis=1))[0]# find(ismember(milestones[:, :2],edges[from_edge_idx, 2:4]))
    # find milestone from edges
    # pt_idx = []
    # for i in range(len(from_edge_idx)):
    #     pt_idx.append(np.all(milestones[:, :2]==edges[from_edge_idx[0][i], 2:4],axis=1).nonzero())# np.where(np.isin(milestones[:, :2], edges[from_edge_idx, 2:4]))[0]

    # Go through all neighbours
    for i in range(len(pt_idx)):

        # # Check if points are visited or not
        # if np.in1d(pt_idx[i], unvisted)[0]:
        #     # remove from list
        #     unvisted = np.delete(unvisted, np.where(unvisted == pt_idx[i]))
            # if pt_idx[i] in unvisted:
            #     #             # remove from list
            #     unvisted = np.delete(unvisted, np.where(unvisted == pt_idx[i]))
        # Calculate g(x) and c(x) between edges
        x_d = milestones[pt_idx[i], 0] - milestones[x_idx, 0]
        y_d = milestones[pt_idx[i], 1] - milestones[x_idx, 1]
        cx = np.sqrt(x_d**2 + y_d**2)
        gx = x_cost + cx

        # only select paths that are more efficient than previous
        if gx < cost_gx[pt_idx[i]] or pt_idx[i] in unvisted:
            unvisted = np.delete(unvisted, np.where(unvisted == pt_idx[i]))
            # if pt_idx[i] == 1:
            #     print(pt_idx[i])
            #     pass
            cost_gx[pt_idx[i]] = gx
            previous[pt_idx[i]] = x_idx

            # Calculate cost and h(x)
            x_d = milestones[pt_idx[i], 0] - finish[0]
            y_d = milestones[pt_idx[i], 1] - finish[1]
            hx = np.sqrt(y_d**2 + x_d**2)
            cost = gx + hx

            # Adding to Q depends on rank
            if not Q_cost:
                Q_idx.append(pt_idx[i])
                Q_cost.append(cost)
            else:
                id = find(Q_cost < cost)

                if not id:  # smallest value
                    Q_cost = [cost] + Q_cost
                    Q_idx = [pt_idx[i]] + Q_idx
                else:  # In the middle
                    id = id[-1]
                    Q_cost = Q_cost[:id+1] + [cost] + Q_cost[id+1:]
                    Q_idx = Q_idx[:id+1] + [pt_idx[i]] + Q_idx[id+1:]

# create the shortest path
if success:
    idx = 1
    spath = [1]
    while idx != 0:
        idx = int(previous[idx][0])
        spath = [int(previous[idx][0])] + spath
else:
    print("No path exists")

# ------end of shortest path finding algorithm-------

#------end of your optimized algorithm-------


elapsed = time.time() - t2
print("Solved in Elapsed time is ",elapsed, " seconds.")

# plot the shortest path
h = plt.figure(1)
plt.clf()
plt.plot(start[0], start[1], 'go')
plt.plot(finish[0], finish[1], 'rx')
plt.plot(milestones[:,0], milestones[:,1], 'm.')
if len(edges) != 0:
    plt.plot(edges[:, [0,2]].T, edges[:, [1,3]].T, color='magenta')
for i in range(len(path)):
    plt.plot(milestones[path[i:i+1], 0], milestones[path[i:i+1], 1],  'go-', linewidth=3)
for i in range(len(spath)):
    plt.plot(milestones[spath[i:i+1], 0], milestones[spath[i:i+1], 1],  'ro-', linewidth=3)
str = "Q3 - {} X {} Maze solved in {} seconds".format(row, col, elapsed)
plt.title(str)
show_maze.show_maze(maze_map,row,col,h) # Draws the maze
# plt.savefig("assignment1_q3.png")
plt.show()