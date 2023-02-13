import numpy as np
def EdgePtsToVec(edge):
    f = np.array([edge[0], edge[1], 0])
    g = np.array([edge[2], edge[3], 0])

    A = np.cross(f-g, [0,0,1])
    A = A / np.linalg.norm(A)
    b = np.dot(A, f)
    A = A[:2]

    return A, b

# e1 = dict()
# e2 = dict()
# e1['A'], e1['b'] = EdgePtsToVec([0, 0, 1, 1])
# print(e1['A'],  e1['b'])