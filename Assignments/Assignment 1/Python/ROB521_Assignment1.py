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
import numpy as np
import matplotlib.pyplot as plt
import pyximport; pyximport.install()
import maze
import show_maze
import AStar
import PRM
# set random seed for repeatability if desired
#np.random.seed(1337)

# ==========================
# Maze Generation
# ==========================
#
# The maze function returns a map object with all of the edges in the maze.
# Each row of the map structure draws a single line of the maze.  The
# function returns the lines with coordinates [x1 y1 x2 y2].
# Bottom left corner of maze is [0.5 0.5],
# Top right corner is [col+0.5 row+0.5]


row = 5 # Maze rows
col = 7 # Maze columns
maze_map = maze.maze(row,col) # Creates the maze
start = [0.5, 1.0] # Start at the bottom left
finish = [col+0.5, row] # Finish at the top right


h = plt.figure(1)
plt.clf()
plt.plot(start[0], start[1], 'go')
plt.plot(finish[0], finish[1], 'rx')
show_maze.show_maze(maze_map,row,col,h) # Draws the maze
plt.show()

# ======================================================
# Question 1: construct a PRM connecting start and finish
# ======================================================
#
# Using 500 samples, construct a PRM graph whose milestones stay at least
# 0.1 units away from all walls, using the MinDist2Edges function provided for
# collision detection.  Use a nearest neighbour connection strategy and the
# CheckCollision function provided for collision checking, and find an
# appropriate number of connections to ensure a connection from  start to
# finish with high probability.


# variables to store PRM components
nS = 500  # number of samples to try for milestone creation
milestones = [start, finish]  # each row is a point [x y] in feasible space
edges = []  # each row is should be an edge of the form [x1 y1 x2 y2]

print("Time to create PRM graph")
t = time.time()

# ------insert your PRM generation code here-------
[milestones, edges] = PRM.PRM_Random(col,row,milestones, edges, maze_map, nS)
# ------end of your PRM generation code -------

elapsed = time.time() - t
print("Elapsed time is ",elapsed, " seconds.")

h = plt.figure(1)
plt.clf()
plt.plot(start[0], start[1], 'go')
plt.plot(finish[0], finish[1], 'rx')
plt.plot(milestones[:,0], milestones[:,1], 'm.')
if len(edges) != 0:
    plt.plot(edges[:, [0,2]].T, edges[:, [1,3]].T, color='magenta')
plt.title(f'Q1 - {row} X {col} Maze PRM')
show_maze.show_maze(maze_map,row,col,h) # Draws the maze
plt.savefig("assignment1_q1.png")
plt.show()

# =================================================================
# Question 2: Find the shortest path over the PRM graph
# =================================================================
#
# Using an optimal graph search method (Dijkstra's or A*) , find the
# shortest path across the graph generated.  Please code your own
# implementation instead of using any built in functions.

print('Time to find shortest path')
t = time.time()

# ------insert your shortest path finding algorithm here-------
spath = AStar.AStar(milestones, edges, start, finish)
# ------end of shortest path finding algorithm-------

elapsed = time.time() - t
print("Elapsed time is ",elapsed, " seconds.")

# plot the shortest path
h = plt.figure(1)
plt.clf()
plt.plot(start[0], start[1], 'go')
plt.plot(finish[0], finish[1], 'rx')
plt.plot(milestones[:,0], milestones[:,1], 'm.')
if len(edges) != 0:
    plt.plot(edges[:, [0,2]].T, edges[:, [1,3]].T, color='magenta')
for i in range(len(spath)):
    plt.plot(milestones[spath[i:i+2], 0], milestones[spath[i:i+2], 1],  'go-', linewidth=3)
str = "Q2 - {} X {} Maze Shortest Path".format(row, col)
plt.title(str)
show_maze.show_maze(maze_map,row,col,h) # Draws the maze
plt.savefig("assignment1_q2.png")
plt.show()

# ================================================================
# Question 3: find a faster way
# ================================================================
#
# Modify your milestone generation, edge connection, collision detection
# and/or shortest path methods to reduce runtime.  What is the largest maze
# for which you can find a shortest path from start to goal in under 20
# seconds on your computer? (Anything larger than 40x40 will suffice for
# full marks)

row = 40 # Maze rows
col = 40 # Maze columns
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
[milestones, edges] = PRM.PRM_Uniform(col,row,milestones, edges, maze_map)
# ------end of your PRM generation code -------

elapsed = time.time() - t1
print("PRM graph generation elapsed time is ",elapsed, " seconds.")
print('Time to find shortest path')
t2 = time.time()

# ------insert your shortest path finding algorithm here-------
spath = AStar.AStar(milestones, edges, start, finish)
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
# for i in range(len(path)):
#     plt.plot(milestones[path[i:i+1], 0], milestones[path[i:i+1], 1],  'go-', linewidth=3)
for j in range(len(spath)):
    plt.plot(milestones[spath[j:j+2], 0], milestones[spath[j:j+2], 1],  'go-', linewidth=3)
str = "Q3 - {} X {} Maze solved in {} seconds".format(row, col, elapsed)
plt.title(str)
show_maze.show_maze(maze_map,row,col,h) # Draws the maze
plt.savefig("assignment1_q3.png")
plt.show()