import numpy as np
def MinDist2Edges(points, edges):
    n = len(points[:,0])
    m = len(edges[:,0])
    dist = np.zeros((n,m))
    for ii in range(n):
        P = points[ii, :]
        for jj in range(m):
            Q1 = edges[jj, 0:2]
            Q2 = edges[jj, 2:4]
            l2 = np.linalg.norm(Q2 - Q1) ** 2
            t = np.dot(P - Q1, Q2 - Q1) / l2
            if t < 0.0:
                dist[ii, jj] = np.linalg.norm(P - Q1)
            elif t > 1.0:
                dist[ii, jj] = np.linalg.norm(P - Q2)
            else:
                dist[ii, jj] = np.abs(np.linalg.det(np.vstack(([Q2 - Q1], [P - Q1])))) / np.linalg.norm(Q2 - Q1)

    d = np.min(dist, axis=1)
    return d

# import time
# t1 = time.time()
# print(MinDist2Edges(np.array([[1,2],[1,1]]).reshape((2,2)),  np.array([1,0.5,2,0.5]).reshape((1,4))))
# print(MinDist2Edges(np.array([[0,0],[0,1]]).reshape((2,2)),  np.array([0,0.5,2,0.5]).reshape((1,4))))
# t2 = time.time()
# print(t2-t1)
# import pyximport; pyximport.install()
# import MinDist2Edges2
# t3 = time.time()
# print(MinDist2Edges2.MinDist2Edges(np.array([[1,2],[1,1]]).reshape((2,2)),  np.array([1,0.5,2,0.5]).reshape((1,4))))
# print(MinDist2Edges2.MinDist2Edges(np.array([[0,0],[0,1]]).reshape((2,2)),  np.array([0,0.5,2,0.5]).reshape((1,4))))
# t4 = time.time()
# print(t4-t3)