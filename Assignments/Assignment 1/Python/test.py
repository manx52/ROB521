import numpy as np

edges = np.array([[1, 2, 7, 8], [3, 4, 5, 6], [5, 6, 7, 8]])
milestones = np.array([[3,4],[1, 2], [7, 8]])

# convert each edge to a set of points
edge_points = np.hstack((edges[:, :2], edges[:, 2:]))

# find all points in milestones that are connected to another by an edge
direct_connection = np.isin(edges[:,0:2], milestones[2,:]).all(axis=1).ravel().nonzero()[0]
direct_connection2 = np.isin(edges[:,2:4], milestones[2,:]).all(axis=1).ravel().nonzero()[0]
direct_connection = np.concatenate((direct_connection,direct_connection2))
print(direct_connection)
print("edges: ", edges[direct_connection, 2:4])
direct_connection2 = np.isin(milestones[:,:2], edges[direct_connection, 2:4]).all(axis=1).ravel().nonzero()[0]
direct_connection3 = np.isin(milestones[:,:2], edges[direct_connection, 0:2]).all(axis=1).ravel().nonzero()[0]
direct_connection2 = np.concatenate((direct_connection2,direct_connection3))
print(direct_connection2)
# check if a direct connection exists between milestones via edges
# if np.count_nonzero(direct_connection) == 2:
#     print(True)
# else:
#     print(False)