function [path,minCost]=Astar(vertices, edges)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% compute the cost of each node
cost_node = 0;
for n = 2:size(vertices,1)-1
    delta_y = vertices(end,2) - vertices(n,2);
    delta_x = vertices(end,1) - vertices(n,1);
    dist2goal = sqrt(delta_y^2 + delta_x^2);
    cost_node = [cost_node, dist2goal];
end
cost_node = [cost_node, 0];

% compute the length of edges
len_edges = [];
for n = 1:size(edges, 1)
    delta_y = vertices(edges(n,1),2) - vertices(edges(n,2),2);
    delta_x = vertices(edges(n,1),1) - vertices(edges(n,2),1);
    len_edges(n) = sqrt(delta_y^2 + delta_x^2);
end

% initialize the O list and C list
C_list = [1, 0];
i = find(edges(:,1)==1);
O_node = edges(i(1):i(end),2);
O_list = [];
for n = 1:size(O_node,1)
    O_list(n,1) = O_node(n,1);
    inx1 = find(edges(:,1)==1);
    inx2 = find(edges(inx1,2)==O_node(n,1));
    O_list(n,2) = cost_node(O_node(n)) + len_edges(inx1(inx2));
end
% sort the O list by cost
[cost,node] = sort(O_list(:,2));
O_list_sort = [];
for n = 1:size(O_list,1)
    O_list_sort(n,1) = O_list(node(n),1);
    O_list_sort(n,2) = cost(n,1);
    O_list_sort(n,3) = 1; 
end
O_list = O_list_sort;

% update the C list and O list
while O_list(1,1) ~= size(vertices,1)
    % add the least cost node into the C list
    C_list = [C_list; O_list(1,1), O_list(1,3)];
    % search the following nodes in the path
    i = find(edges(:,1)==C_list(end,1));
    add_node = edges(i,2);
    
    if isempty(add_node)==1        
        C_list = [C_list; O_list(1,1), O_list(1,3)];
        continue
    end
    
    % compute the cost of the new explored nodes
    add_list = [];
    for n = 1:size(add_node,1)
        add_list(n,1) = add_node(n);
        add_list(n,2) = cost_node(add_node(n)) + len_edges(i(n)) ...
            + O_list(1,2) - cost_node(O_list(1,1));
        add_list(n,3) = C_list(end,1);
    end
    O_list(1,:) = [];
    O_list = [O_list; add_list];

    % sort the updated O list
    [cost1,node1] = sort(O_list(:,2));
    
    O_list_sort = [];
    for n = 1:size(O_list,1)
        O_list_sort(n,1) = O_list(node1(n,1),1);
        O_list_sort(n,2) = cost1(n,1);
        O_list_sort(n,3) = O_list(node1(n,1),3);
    end
    O_list = O_list_sort;
    
end
% add the goal into the C list
C_list = [C_list; O_list(1,1), C_list(end,1)];

% extract the path
path = [C_list(end,2), C_list(end,1)];
for i = size(C_list,1):-1:1
    if C_list(i,1) == path(1)
        path = [C_list(i,2), C_list(i,1) ,path];
    end
end
path(1) = [];
path = unique(path);

% compute the minimum cost 
minCost = 0;
for n = 1:size(path,2)-1
    dist = sqrt((vertices(path(n),1)-vertices(path(n+1),1))^2 + (vertices(path(n),2)-vertices(path(n+1),2))^2);
    minCost = minCost + dist;
end

end

