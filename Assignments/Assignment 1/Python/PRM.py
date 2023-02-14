import numpy as np
import MinDist2Edges
import CheckCollision
def PRM_Uniform(col,row,milestones, edges, maze_map):
    # 1. Generate N points uniformally in maze boundary
    pts = []
    for i in range(1, col + 1):
        for j in range(1, row + 1):
            pts.append([i, j])

    nS = len(pts)
    pts = np.array(pts)


    # 2.remove collided points if its 0.1 nearest to a wall
    pts = CheckPoints(pts, maze_map, nS)
    milestones = np.concatenate((milestones, pts), axis=0)

    # 3. Find nearest 10 neighbors and link them
    edges = Knn(6, milestones, edges, maze_map)

    return milestones, edges

def PRM_Random(col,row,milestones, edges, maze_map, nS):
    # 1. Generate N random points in maze boundary
    x_high = col + 0.5
    x_low = 0.5
    y_high = row + 0.5
    y_low = 0.5
    x = (x_high - x_low) * np.random.random((nS, 1)) + x_low
    y = (y_high - y_low) * np.random.random((nS, 1)) + y_low
    pts = np.array([x, y]).T[0]

    # 2.remove collided points if its 0.1 nearest to a wall
    pts = CheckPoints(pts, maze_map, nS)
    milestones = np.concatenate((milestones, pts), axis=0)

    # 3. Find nearest 10 neighbors and link them
    edges = Knn(12, milestones, edges, maze_map)

    return milestones, edges

def Knn(k, milestones, edges, maze_map):
    for i in range(len(milestones)):
        x_d = milestones[i, 0] - milestones[:, 0]
        y_d = milestones[i, 1] - milestones[:, 1]
        distance = np.sqrt(x_d ** 2 + y_d ** 2)
        sorted_indices = np.argsort(distance)

        for j in range(k):
            [inCollision, edge] = CheckCollision.CheckCollision(milestones[i, :], milestones[sorted_indices[j], :],
                                                                maze_map)
            if inCollision == 0:  # check for collision in the edge path
                edges.append(np.concatenate((milestones[i, :], milestones[sorted_indices[j], :]), axis=0))

    edges = np.array(edges)
    return edges

def CheckPoints(pts, maze_map, nS):
    dist = MinDist2Edges.MinDist2Edges(pts, maze_map)
    indices = []
    for i in range(nS):
        if dist[i] < 0.1:
            indices.append(i)

    pts = np.delete(pts, indices, axis=0)
    return pts