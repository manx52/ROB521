import numpy as np
def maze(row, col):
    # seed

    [cc, rr] = np.meshgrid(np.arange(1, col + 1, 1), np.arange(1, row + 1, 1))
    state = np.arange(1, (row * col) + 1, 1).reshape(col, row).T  # state identifies connected regions
    id_ = np.arange(1, (row * col) + 1, 1).reshape(col, row).T  # id identifies intersections of maze

    # create pointers to adjacent intersections
    ptr_left = np.zeros(id_.shape)
    ptr_up = np.zeros(id_.shape)
    ptr_right = np.zeros(id_.shape)
    ptr_down = np.zeros(id_.shape)

    ptr_left[:, 1: id_.shape[1]] = id_[:, 0: id_.shape[1] - 1]
    ptr_up[1: id_.shape[0], :] = id_[0: id_.shape[0] - 1, :]
    ptr_right[:, 0: id_.shape[1] - 1] = id_[:, 1: id_.shape[1]]
    ptr_down[0: id_.shape[0] - 1, :] = id_[1: id_.shape[0], :]

    # sort graph entities by id
    the_maze = np.array(
        [id_.T.flatten(),
         rr.T.flatten(),
         cc.T.flatten(),
         state.T.flatten(),
         ptr_left.T.flatten(),
         ptr_up.T.flatten(),
         ptr_right.T.flatten(),
         ptr_down.T.flatten()]).T

    id_ = the_maze[:, 0]
    rr = the_maze[:, 1]
    cc = the_maze[:, 2]
    state = the_maze[:, 3]
    ptr_left = the_maze[:, 4]
    ptr_up = the_maze[:, 5]
    ptr_right = the_maze[:, 6]
    ptr_down = the_maze[:, 7]

    [state, ptr_left, ptr_up, ptr_right, ptr_down] = make_pattern(row, col, id_, rr, cc, state, ptr_left, ptr_up,
                                                                    ptr_right, ptr_down)

    # make map
    maze_map = [.5,col+.5,.5,.5]
    maze_map = np.vstack((maze_map, [.5, col + .5, row + .5, row + .5]))
    maze_map = np.vstack((maze_map, [.5, .5, 1.5, row + .5]))
    maze_map = np.vstack((maze_map, [col + .5, col + .5, .5, row - .5]))

    for i in range(len(ptr_right)):
        if ptr_right[i] > 0: # right passage blocked
            maze_map = np.vstack((maze_map,[cc[i]+.5,cc[i]+.5,rr[i]-.5,rr[i]+.5]))
        if ptr_down[i] > 0: # right passage blocked
            maze_map = np.vstack((maze_map,[cc[i]-.5,cc[i]+.5,rr[i]+.5,rr[i]+.5]))

    maze_map = np.vstack((maze_map[:,0],maze_map[:,2],maze_map[:,1],maze_map[:,3])).T
    # print(maze_map)
    return maze_map

def make_pattern(row, col, id, rr, cc, state, ptr_left, ptr_up, ptr_right, ptr_down):
    while np.max(state) > 1:
        tid = (np.ceil(((col * row) * np.random.rand(15, 1)))).astype(int)  # get a set of temporary ID's
        # tid_1 = np.array([2,25,15,15,0,28,12,29,1,34,5,8,21,8,8]) #for testing
        # tid = tid_1 + 1
        cityblock = cc[tid - 1] + rr[tid - 1]  # get distance from the start
        is_linked = (state[tid - 1] == 1)  # The start state is in region 1 - see if they are linked to the start
        # temp = sortrows(cat(2, tid, cityblock, is_linked), [3, 2])
        temp = np.array([tid, cityblock, is_linked]).T[0]
        temp_idx = np.lexsort((temp[:, 1], temp[:, 2]))
        temp = temp[temp_idx]  # sort id's by start-link and distance
        tid = (temp[0, 0] - 1).astype(int)

        direct = np.ceil(8 * np.random.rand())
        nb = 3
        block_size = min(row, col) / nb
        while block_size > 12:
            nb += 2
            block_size = min(row, col) / nb

        odd_even = (np.ceil(rr[tid] / block_size) * np.ceil(col / block_size) + np.ceil(cc[tid] / block_size))

        if odd_even / 2 == np.floor(odd_even / 2):
            if direct > 6:
                direct = 4
            if direct > 4:
                direct = 3
        else:
            if direct > 6:
                direct = 2
            if direct > 4:
                direct = 1
        if direct == -1:
            pass
        elif direct == 1:
            tmp = ptr_left[tid].astype(int) - 1
            if ptr_left[tid] > 0 and state[tid] != state[tmp]:
                state[(state == state[tid]) | (state == state[tmp])] = min(state[tid], state[tmp])
                ptr_right[tmp] = -1 * ptr_right[tmp]
                ptr_left[tid] = -1 * ptr_left[tid]
        elif direct == 2:
            tmp = ptr_right[tid].astype(int)- 1
            if ptr_right[tid] > 0 and state[tid] != state[tmp]:
                state[(state == state[tid]) | (state == state[tmp])] = min(state[tid], state[tmp])
                ptr_left[tmp] = -1 * ptr_left[tmp]
                ptr_right[tid] = -1 * ptr_right[tid]
        elif direct == 3:
            tmp = ptr_up[tid].astype(int)- 1
            if ptr_up[tid] > 0 and state[tid] != state[tmp]:
                state[(state == state[tid]) | (state == state[tmp])] = min(state[tid], state[tmp])
                ptr_down[tmp] = -1 * ptr_down[tmp]
                ptr_up[tid] = -1 * ptr_up[tid]
        elif direct == 4:
            tmp = ptr_down[tid].astype(int)- 1
            if ptr_down[tid] > 0 and state[tid] != state[tmp]:
                state[(state == state[tid]) | (state == state[tmp])] = min(state[tid], state[tmp])
                ptr_up[tmp] = -1 * ptr_up[tmp]
                ptr_down[tid] = -1 * ptr_down[tid]
        else:
            print(direct)
            raise ValueError("quit")

        # print(max(state))
    return [state, ptr_left, ptr_up, ptr_right, ptr_down]

