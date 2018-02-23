function [vertices,edges,path]=rrt(map,q_start,q_goal,k,delta_q,p)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% Initialize the vertices and edges variables
vertices = q_start;
edges = [];

x_max = size(map,2);
y_max = size(map,1);

for n = 1:k
    % With a certain probability to choose q_goal as q_rand
    if rand() <= p
        q_rand = q_goal;
    else
        q_rand = [x_max*rand(), y_max*rand()];
    end
    
    % Search q_near in the vertices
    dist = [];
    for i = 1:size(vertices,1)
        dist(i) = sqrt((q_rand(2)-vertices(i,2))^2+(q_rand(1)-vertices(i,1))^2);
    end
    [~, q_near_idx] = min(dist);
    q_near = vertices(q_near_idx,:);
    
    % Generate q_new at delta_q distance from q_near in the direction to q_rand
    q_new = [];
    step_size = min(delta_q, norm(q_rand - q_near));
    q_new = q_near + step_size*(q_rand-q_near)/norm(q_rand-q_near);
    
    % Check if q_new belongs to free space
    if q_new(1)>=1 && q_new(1)<=x_max && q_new(2)>=1 && q_new(2)<=y_max
        if map(round(q_new(2)), round(q_new(1))) == 0
            % Check if the edge belongs to the free space
            num_segm = 20;        % number of segments
            delta_x = (q_new(1) - q_near(1))/num_segm;
            delta_y = (q_new(2) - q_near(2))/num_segm;
            
            checkPt_x = q_near(1) + delta_x;
            checkPt_y = q_near(2) + delta_y;
            
            occupied_flag = 0;
            for s = 1:num_segm-1
                if map(round(checkPt_y), round(checkPt_x)) == 1
                    occupied_flag = 1;
                    break
                else
                    checkPt_x = checkPt_x + delta_x;
                    checkPt_y = checkPt_y + delta_y;
                end
            end
            if occupied_flag == 1
                continue
            end
            
            % Add q_new in the vertices
            vertices = [vertices; q_new];
            % Add [index(q_new) index(q_near)] in edges
            edges = [edges; size(vertices,1), q_near_idx];
            
            if q_new == q_goal
                disp(['Found goal on iteration ', num2str(n)])
                break
            end
        end
        
    end
end

disp(['Finished on iteration ', num2str(n)])

path = edges(size(edges,1),:);
while path(end) ~= 1
    next_vertex = find(edges(:,1) == path(end));
    path = [path, edges(next_vertex,2)];
end

end

