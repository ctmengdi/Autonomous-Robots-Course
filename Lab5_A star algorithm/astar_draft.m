% ver_x = [0.7807; 3.0322; 1.3655; 4.1140; 6.2778;
%     8.2953; 5.6345; 9.1433; 11.4825; 10.2544];
% ver_y = [9.0497; 8.9912; 6.7105; 4.0497; 8.2310;
%     5.8333; 2.6170; 1.9152; 6.9444; 0.5702];
% inx = [0; 1; 1; 1; 1; 2; 2; 2; 2; 3];
% vertices = [ver_x,ver_y,inx];
vertices=[0.6053, 7.9971, 0;
1.0439, 6.8567, 1.0000;
2.9737, 8.2602, 1.0000;
3.9386, 6.3304, 1.0000;
1.9795, 5.3655, 1.0000;
6.4532, 8.3187, 2.0000;
5.1959, 6.6228, 2.0000;
6.3070, 4.6637, 2.0000;
8.7339, 6.2719, 2.0000;
8.4708, 7.8801, 2.0000;
3.2368, 4.8684, 3.0000;
0.8684, 3.9620, 3.0000;
1.2485, 2.5585, 3.0000;
3.3538, 2.4123, 3.0000;
4.8450, 4.0497, 3.0000;
6.5994, 3.9327, 4.0000;
6.5409, 2.0906, 4.0000;
8.5877, 2.2076, 4.0000;
8.6170, 4.6053, 4.0000;
9.4357, 7.2368, 5.0000;
11.1608, 4.0789, 5.0000;
10.3129, 7.9094, 5.0000;
10.3713, 1.5351, 6.0000];
edges = RPS(vertices);

% compute the cost of each node
cost_node = 0;
for n = 2:size(vertices,1)-1
    delta_y = vertices(end,2) - vertices(n,2);
    delta_x = vertices(end,1) - vertices(n,1);
    dist2goal = sqrt(delta_y^2 + delta_x^2);
    cost_node = [cost_node, dist2goal];
end
cost_node = [cost_node, 0];
%disp(cost_node);

% compute the length of edges
len_edges = [];
for n = 1:size(edges, 1)
    delta_y = vertices(edges(n,1),2) - vertices(edges(n,2),2);
    delta_x = vertices(edges(n,1),1) - vertices(edges(n,2),1);
    len_edges(n) = sqrt(delta_y^2 + delta_x^2);
end
%disp(len_edges);

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
[cost,node] = sort(O_list(:,2));
% disp(O_list);
% disp(node);
O_list_sort = [];
for n = 1:size(O_list,1)
    O_list_sort(n,1) = O_list(node(n),1);
    O_list_sort(n,2) = cost(n,1);
    O_list_sort(n,3) = 1; 
end
O_list = O_list_sort;
disp(C_list);
disp(O_list);

while O_list(1,1) ~= size(vertices,1)
    disp(C_list);
    disp(O_list);
    C_list = [C_list; O_list(1,1), O_list(1,3)];
    i = find(edges(:,1)==C_list(end,1));
    add_node = edges(i,2);
    
    if isempty(add_node)==1        
        C_list = [C_list; O_list(1,1), O_list(1,3)];
        continue
    end
    
    % continue exploring new node
    add_list = [];
    for n = 1:size(add_node,1)
        add_list(n,1) = add_node(n);
        add_list(n,2) = cost_node(add_node(n)) + len_edges(i(n)) ...
            + O_list(1,2) - cost_node(O_list(1,1));
        add_list(n,3) = C_list(end,1);
    end
    O_list(1,:) = [];
    O_list = [O_list; add_list];
    %disp(O_list);
    % sort the O list
    [cost1,node1] = sort(O_list(:,2));
    
    O_list_sort = [];
    for n = 1:size(O_list,1)
        O_list_sort(n,1) = O_list(node1(n,1),1);
        O_list_sort(n,2) = cost1(n,1);
        O_list_sort(n,3) = O_list(node1(n,1),3);
    end
    O_list = O_list_sort;
    
%     disp(C_list);
%     disp(O_list);
%     i = find(edges(:,1)==C_list(end,1));
%     j = find(edges(i,2)==O_list(1,1));

%     if isempty(j)==0
%         C_list = [C_list; O_list(1,1), C_list(end,1)];
%     else
%         C_list = [C_list; O_list(1,1), 1];
%     end
%     inx = find(edges(:,1)==C_list(end,1));
%     add_list = [];
%     for n = 1:size(inx,1)
%         add_list(n,1) = edges(inx(n),2);
%         add_list(n,2) = cost_node(add_list(n,1)) + len_edges(inx(n))...
%             + O_list(1,2) - cost_node(O_list(1,1));
%     end
%     O_list(1,:) = [];
%     O_list = [O_list ; add_list];
%     %disp(O_list);
%     [cost1,node1] = sort(O_list(:,2));
% %     disp(cost1);
% %     disp(node1);
%     O_list_sort = [];
%     for n = 1:size(O_list,1)
%         O_list_sort(n,1) = O_list(node1(n,1),1);
%         O_list_sort(n,2) = cost1(n,1);
%     end
%     O_list = O_list_sort;
%     disp(O_list);
%     disp(C_list);
end
C_list = [C_list; O_list(1,1), C_list(end,1)];
disp(C_list);
disp(O_list);
%%
path = [C_list(end,2), C_list(end,1)];
%disp(path);
for i = size(C_list,1):-1:1
    if C_list(i,1) == path(1)
        path = [C_list(i,2), C_list(i,1) ,path];
    end
end
path(1) = [];
path = unique(path);

disp(path);

min_cost = 0;
for n = 1:size(path,2)-1
    dist = sqrt((vertices(path(n),1)-vertices(path(n+1),1))^2 + (vertices(path(n),2)-vertices(path(n+1),2))^2);
    min_cost = min_cost + dist;
end
disp(min_cost);

plot_astar(vertices, edges, path)
 










