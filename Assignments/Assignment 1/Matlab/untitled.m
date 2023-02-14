e = [[1 2 7 8]; [3 4 5 6];[5 6 7 8]]
m = [[3,4];[1 2]; [7 8]]

from_edge_idx = find(ismember(e(:, 1:2), m(2,:),'rows'))
e(from_edge_idx,3:4)
pt_idx = find(ismember(m(:,1:2), e(from_edge_idx,3:4),'rows'))