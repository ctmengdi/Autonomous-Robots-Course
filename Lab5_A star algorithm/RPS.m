function [ edges ] = RPS( vertices )
% This is a function of Rotational Plane Sweep algorithm
% input: a set of vertices where first row is srart position and
% the end row is goal. Three columns are x, y coordinates and ascengding
% index which the vertices in same polygons have same indices.
% output: edges of available path
% Implemented by Di MENG at 04/29/2017 for Autonomous Robots lab4, UdG

% extract edges of polygons
edges_polyg = [];
for n = vertices(2,3):vertices(end-1,3)
    polyg = find(vertices(:,3)==n);
    for v = 1:size(polyg,1)
        if v ~= size(polyg,1)
            edges_polyg = [edges_polyg; polyg(v), polyg(v+1)];
        else
            edges_polyg = [edges_polyg; polyg(v), polyg(1)];
        end
    end
end

% visualize the environment - start and goal point
figure; 
plot(vertices(1,1), vertices(1,2),'gx');
hold on;
plot(vertices(end,1), vertices(end,2),'gx');
hold on;

% visualize the environment - polygons
for n = 1:size(edges_polyg,1)
    plot([vertices(edges_polyg(n,1),1), vertices(edges_polyg(n,2),1)],...
        [vertices(edges_polyg(n,1),2), vertices(edges_polyg(n,2),2)],'b');
    hold on;
end
hold off;title('Environment');

% a list of sorted angles related to corresponding vertex
start_vertex = 1;
visible_graph = [];
while start_vertex ~= size(vertices,1)
    angles = [];
    num_ang = 1;
    for n = 1:size(vertices,1)
        if n == start_vertex
            continue
        end
        delta_y = vertices(n, 2)-vertices(start_vertex, 2);
        delta_x = vertices(n, 1)-vertices(start_vertex, 1);
        % fit the angles in range [0,360] degrees
        angles(num_ang,:) = atan2d(delta_y, delta_x) + 360*(delta_y<0);
        num_ang = num_ang + 1;
    end
    [sorted_ang,inx_ang] = sort(angles);
    
    % match the angle index to vertex index
    for n = 1:size(inx_ang,1)
        if inx_ang(n) >= start_vertex
            inx_ang(n) = inx_ang(n) + 1;
        end
    end
    
    % initialize the S list
    S = [];
    for n = 1:size(edges_polyg,1)
        x1 = [vertices(start_vertex,1), max(vertices(:,1))];
        y_h = [vertices(start_vertex, 2), vertices(start_vertex, 2)];
        x2 = [vertices(edges_polyg(n,1),1), vertices(edges_polyg(n,2),1)];
        y = [vertices(edges_polyg(n,1),2), vertices(edges_polyg(n,2),2)];
        % find if there's intersect of horizontal line and all the edges
        [intersec_x, intersec_y] = polyxpoly(x1,y_h,x2,y);
        if isempty([intersec_x, intersec_y])==0
            S = [S,n];
        end
    end
    
    % delete the connecting edges in the initial S list when
    % the start vertex is in the polygon
    if start_vertex ~= 1 && start_vertex ~= size(vertices,1)
        [i, ~] = find(edges_polyg == start_vertex);
        edge_i = find(S==i(1));
        edge_j = find(S==i(2));
        remove = [edge_i edge_j];
        S(remove) = [];
    end
    
    % decide if the vertex is visible
    for n = 1:size(inx_ang,1)
        % check if there's intersect between start vertex-current vertex and
        % every edge in the S list
        intersect = [];
        if isempty(S) == 1
            visible_graph = [visible_graph; start_vertex, inx_ang(n)];
        else
            for s = 1:size(S,2)
                x1 = [vertices(start_vertex, 1), vertices(inx_ang(n), 1)];
                y1 = [vertices(start_vertex, 2), vertices(inx_ang(n), 2)];
                x2 = [vertices(edges_polyg(S(s),1),1), vertices(edges_polyg(S(s),2),1)];
                y2 = [vertices(edges_polyg(S(s),1),2), vertices(edges_polyg(S(s),2),2)];
                [intersect_x, intersect_y] = polyxpoly(x1,y1,x2,y2);
                intersect = [intersect; intersect_x, intersect_y];
            end
            % delete the repeat rows
            intersect = unique(intersect, 'rows');
            
            % the conditions when the vertex is visible
            flag1 = 0;
            if size(intersect,1) ==2
                int_ = [intersect(1,:), intersect(2,:)];
                line_ = [vertices(start_vertex,1:2), vertices(inx_ang(n),1:2)];
                int_s = sort(int_);
                line_s = sort(line_);
                if isequal(int_s, line_s) ==1
                    flag1 = 1;
                end
            end
            flag2 = 0;
            if size(intersect,1) ==1
                int_ = [intersect(1), intersect(2)];
                endPt1 = vertices(start_vertex, 1:2);
                endPt2 = vertices(inx_ang(n), 1:2);
                if isequal(int_,endPt1) == 1 || isequal(int_,endPt2) == 1
                    flag2 = 1;
                end
            end
            if isempty(intersect)==1  || flag1 == 1 || flag2 == 1
                visible_graph = [visible_graph; start_vertex, inx_ang(n)];
            end
        end
        
        % update S list
        if inx_ang(n) ~= size(vertices,1) && inx_ang(n) ~= 1
            % extract the connecting edges of current vertex
            [i, ~] = find(edges_polyg == inx_ang(n));
            if isempty(S) == 1
                S = [S, i(1), i(2)];
            else
                edge_i = find(S==i(1));
                edge_j = find(S==i(2));
                if isempty(edge_i) == 1 && isempty(edge_j)==1
                    S = [S, i(1), i(2)];
                end
                if isempty(edge_i) == 0 && isempty(edge_j)==0
                    remove = [edge_i edge_j];
                    S(remove) = [];
                end
                if isempty(edge_i) == 0 && isempty(edge_j)==1
                    S(edge_i) = [];
                    S = [S, i(2)];
                end
                if isempty(edge_i) == 1 && isempty(edge_j)==0
                    S(edge_j) = [];
                    S = [S, i(1)];
                end
            end
        end
    end
    
    start_vertex = start_vertex + 1;
end

% remove repeating edges
tobeRemove1 = [];
for n = 1:size(visible_graph,1)
    if visible_graph(n,2) < visible_graph(n,1)
        tobeRemove1 = [tobeRemove1; n];
    end
end
visible_graph(tobeRemove1,:) = [];

% remove the edges which are inside the polygons(invisible)
tobeRemove2 = [];
for n = 1:size(visible_graph,1)
    if visible_graph(n,2) - visible_graph(n,1) > 1 && ...
            vertices(visible_graph(n,2),3) == vertices(visible_graph(n,1),3)
        % the center point of the edge
        x_center = (vertices(visible_graph(n,1),1) + vertices(visible_graph(n,2),1))/2;
        y_center = (vertices(visible_graph(n,1),2) + vertices(visible_graph(n,2),2))/2;
        ver = find(vertices(:,3)==vertices(visible_graph(n,2),3));
        % x, y vectors of polygons
        x_polyg = [];
        y_polyg = [];
        for v = 1:size(ver,1)
            x_polyg = [x_polyg, vertices(ver(v),1)];
            y_polyg = [y_polyg, vertices(ver(v),2)];
        end
        % decide if the center point is in/on the polygon
        [in, on] = inpolygon(x_center,y_center,x_polyg,y_polyg);
        if in == 1 && on == 0
            tobeRemove2 = [tobeRemove2; n];
        end
        
    end
end
visible_graph(tobeRemove2,:) = [];
edges = visible_graph;

% visualize the visible graph
figure; 
for n = 1:size(edges,1)
    plot([vertices(edges(n,1),1), vertices(edges(n,2),1)],...
        [vertices(edges(n,1),2), vertices(edges(n,2),2)],'r');
    hold on;
end

% visualize the start and goal point
plot(vertices(1,1), vertices(1,2),'gx');
hold on;
plot(vertices(end,1), vertices(end,2),'gx');
hold on;

% visualize the polygons
for n = 1:size(edges_polyg,1)
    plot([vertices(edges_polyg(n,1),1), vertices(edges_polyg(n,2),1)],...
        [vertices(edges_polyg(n,1),2), vertices(edges_polyg(n,2),2)],'b');
    hold on;
end
hold off;title('Visible graph');

end

