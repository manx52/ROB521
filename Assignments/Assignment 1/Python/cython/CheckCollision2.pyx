import numpy as np
import pyximport; pyximport.install()
import EdgeCollision2
def CheckCollision(ptA, ptB, obstEdges):
    edge = []
    for k in range(obstEdges.shape[0]):
        if not ((max(ptA[0], ptB[0]) < min(obstEdges[k,0], obstEdges[k,2])) or
                (min(ptA[0], ptB[0]) > max(obstEdges[k,0], obstEdges[k,2])) or
                (max(ptA[1], ptB[1]) < min(obstEdges[k,1], obstEdges[k,3])) or
                (min(ptA[1], ptB[1]) > max(obstEdges[k,1], obstEdges[k,3]))):
            if EdgeCollision2.EdgeCollision(np.concatenate((ptA, ptB), axis=None), obstEdges[k, :]):
                if (sum(abs(ptA - obstEdges[k,0:2])) > 0 and
                    sum(abs(ptB - obstEdges[k,0:2])) > 0 and
                    sum(abs(ptA - obstEdges[k,2:4])) > 0 and
                    sum(abs(ptB - obstEdges[k,2:4])) > 0):
                    edge = k
                    inCollision = 1
                    return inCollision, edge
    inCollision = 0
    return inCollision, edge


# print(CheckCollision([1,2],[1,1],  np.array([1,0.5,2,0.5]).reshape((1,4))))
# print(CheckCollision([0,0],[0,1],  np.array([0,0.5,2,0.5]).reshape((1,4))))