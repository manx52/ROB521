import numpy as np
def AffineIntersect(edge1, edge2):
    A = np.vstack([edge1['A'], edge2['A']])
    b = np.hstack([edge1['b'], edge2['b']])

    A = np.minimum(A, 1e6)

    if np.linalg.cond(A) > 1e6:
        return [0, 0], 0
    else:
        pt = np.dot(np.linalg.inv(A), b).T
        return pt.tolist(), 1


# e1 = dict()
# e2 = dict()
# e1['A'], e1['b'] = EdgePtsToVec.EdgePtsToVec([0, 0, 0, 1])
# e2['A'], e2['b'] = EdgePtsToVec.EdgePtsToVec([1, 1, 2, 2])
#
# [pt, colliding] = AffineIntersect(e1,e2)
# print(pt, colliding)