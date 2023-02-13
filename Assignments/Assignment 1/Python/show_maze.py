import matplotlib.pyplot as plt
import numpy as np
def show_maze(maze_map,row,col,h):
    maze_map = np.column_stack((maze_map[:,0], maze_map[:,2], maze_map[:,1], maze_map[:,3]))
    plt.figure(h)

    for ii in range(len(maze_map[:, 0])):
        plt.plot(maze_map[ii, 0:2], maze_map[ii, 2:4])

    plt.axis('equal')
    plt.axis([.5, col + .5, .5, row + .5])
    plt.axis('off')

    return
