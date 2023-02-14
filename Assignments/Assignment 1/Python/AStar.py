import numpy as np
def find(lst):
    return [j for j, x in enumerate(lst) if x]

def AStar(milestones, edges, start, finish):
    unvisted = np.arange(1, len(milestones), 1)  # removed start
    spath = []  # shortest path, stored as a milestone row index sequence
    cost_gx = float("inf") * np.ones((len(milestones), 1))

    # Calculate h(x) for start
    x_d = start[0] - finish[0]
    y_d = start[1] - finish[1]
    hx = np.sqrt(y_d ** 2 + x_d ** 2)
    cost = 0 + hx

    # Tracks previous obtained milestone
    previous = -1 * np.ones((len(milestones), 1))  # np.full((len(milestones), 1), -1)

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

        # Found goal
        if x_idx == 1:
            success = True
            break

        # Get edges
        from_edge_idx = np.where(np.equal(edges[:, :2], milestones[x_idx]).all(axis=1))[0]

        # find milestone from edges
        pt_idx = []
        for pt in from_edge_idx:
            temp = np.equal(milestones[:, :2], edges[pt, 2:4])
            pt_idx.append((np.where(temp[:, 0] & temp[:, 1])[0][0]))

        # Go through all neighbours
        for i in range(len(pt_idx)):
            x_d = milestones[pt_idx[i], 0] - milestones[x_idx, 0]
            y_d = milestones[pt_idx[i], 1] - milestones[x_idx, 1]
            cx = np.sqrt(x_d ** 2 + y_d ** 2)
            gx = x_cost + cx

            # only select paths that are more efficient than previous
            if gx < cost_gx[pt_idx[i]]:  # or pt_idx[i] in unvisted:
                cost_gx[pt_idx[i]] = gx
                previous[pt_idx[i]] = x_idx

                # Calculate cost and h(x)
                x_d = milestones[pt_idx[i], 0] - finish[0]
                y_d = milestones[pt_idx[i], 1] - finish[1]
                # hx = np.sqrt(y_d**2 + x_d**2)
                hx = abs(y_d) + abs(x_d)
                cost = gx + hx

                # Adding to Q depends on rank
                if not Q_cost:
                    Q_idx.append(pt_idx[i])
                    Q_cost.append(cost)
                else:
                    id_ = find(Q_cost < cost)

                    if not id_:  # smallest value
                        Q_cost = [cost] + Q_cost
                        Q_idx = [pt_idx[i]] + Q_idx
                    else:  # In the middle
                        id_ = id_[-1]
                        Q_cost = Q_cost[:id_ + 1] + [cost] + Q_cost[id_ + 1:]
                        Q_idx = Q_idx[:id_ + 1] + [pt_idx[i]] + Q_idx[id_ + 1:]

                if pt_idx[i] in unvisted:
                    # remove from list
                    unvisted = np.delete(unvisted, np.where(unvisted == pt_idx[i])[0][0])
    # create the shortest path
    if success:
        idx = 1
        spath = [1]
        while idx != 0:
            spath = [int(previous[idx][0])] + spath
            idx = int(previous[idx][0])
    else:
        print("No path exists")

    return spath
