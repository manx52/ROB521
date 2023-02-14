import numpy as np
# import EdgePtsToVec
# import AffineIntersect
import pyximport; pyximport.install()
import EdgePtsToVec2
import AffineIntersect2
def EdgeCollision(Edge1, Edge2, tol=0):
    # EDGECOLLISION Checks if two edges are colliding
    # Edge1 -> First edge being checked
    # Edge2 -> Second edge being checked
    # return-> 1: if colliding
    #          0: if not colliding
    # Edges are represented by endpoints [x1 y1 x2 y2]

    p1 = Edge1[:2]
    p2 = Edge1[2:]
    p3 = Edge2[:2]
    p4 = Edge2[2:]

    # If lines are the same
    if (np.abs(p1[0] - p3[0]) <= tol and np.abs(p1[1] - p3[1]) <= tol and
            np.abs(p2[0] - p4[0]) <= tol and np.abs(p2[1] - p4[1]) <= tol):
        colliding = True
        pt = p1
        return colliding, pt

    # From Introduction to Algorithms, Cormen et. al.

    np.concatenate((p2 - p1, [0]))
    temp = np.vstack((np.concatenate((p1 - p3, [0])), np.concatenate((p2 - p3, [0])),np.concatenate((p3 - p1, [0])), np.concatenate((p4 - p1, [0]))))
    temp1 = np.vstack((np.concatenate((p4 - p3, [0])), np.concatenate((p4 - p3, [0])),np.concatenate((p2 - p1, [0])), np.concatenate((p2 - p1, [0]))))

    d = np.cross(temp.T,temp1.T,axis=0)
    d = d[-1]

    colliding = False
    if ((d[0] > 0 and d[1] < 0) or (d[0] < 0 and d[1] > 0)) and (
            (d[2] > 0 and d[3] < 0) or (d[2] < 0 and d[3] > 0)):
        colliding = True
    elif (np.abs(d[0]) < 100 * np.finfo(float).eps and
          OnSegment(p3, p4, p1)):
        colliding = True
    elif (np.abs(d[1]) < 100 * np.finfo(float).eps and
          OnSegment(p3, p4, p2)):
        colliding = True
    elif (np.abs(d[2]) < 100 * np.finfo(float).eps and
          OnSegment(p1, p2, p3)):
        colliding = True
    elif (np.abs(d[3]) < 100 * np.finfo(float).eps and
          OnSegment(p1, p2, p4)):
        colliding = True

    # Find intersection point
    if colliding:
        e1 = dict()
        e2 = dict()
        e1['A'], e1['b'] = EdgePtsToVec2.EdgePtsToVec(Edge1)
        e2['A'], e2['b'] = EdgePtsToVec2.EdgePtsToVec(Edge2)
        pt, colliding = AffineIntersect2.AffineIntersect(e1, e2)
    else:
        pt = [0, 0]

    return colliding, pt


def OnSegment(pi, pj, pk):
    if (min(pi[0], pj[0]) <= pk[0] and pk[0] <= max(pi[0], pj[0])) and (min(pi[1], pj[1]) <= pk[1] and pk[1] <= max(pi[1], pj[1])):
        return True
    else:
        return False


# print(EdgeCollision(np.array([1,2, 1,1]).reshape((4)),  np.array([1,0.5,2,0.5]).reshape((4))))
# print(EdgeCollision(np.array([0,0,0,1]).reshape((4)),  np.array([0,0.5,2,0.5]).reshape((4))))