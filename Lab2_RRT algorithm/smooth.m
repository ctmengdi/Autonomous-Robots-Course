function [path_smooth]=smooth(map,path,vertices,delta)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% Initialize the path_smooth with the goal point
path_smooth = path(1);

% Index of the start and end point of line
idx_start = path(end);    idx_end = path(1);  idx_move = 1;

% Stops when path_smooth has the first vertex
while path_smooth(end) ~= path(end)
    
    % If the distance between start and end point is less than delta, take
    % the distance as step size
    step_size = delta;
    delta_vec = step_size*(vertices(idx_end,:) - vertices(idx_start,:))/...
        norm(vertices(idx_end,:) - vertices(idx_start,:));
    checkPt = vertices(idx_start,:) + delta_vec;
    occupied_flag = 0;
    
    % Stops when there's no obstacle on the edge
    while norm(vertices(idx_end,:) - checkPt) >= step_size
        % Check collision-free
        if map(round(checkPt(2)), round(checkPt(1))) == 1
            occupied_flag = 1;
            break
        end
        
        %Assign next checkPt
        checkPt = checkPt + delta_vec;
    end
    
    % When there's obstacle on the edge, consider the start point as the
    % next vertex in the path
    if occupied_flag ==1
        idx_start = path(end-idx_move);
        idx_move = idx_move + 1;
        disp(['occupied--idx_start', num2str(idx_start)]);
    else       
        % Put the start point in the path_smooth when it can connect to the end
        path_smooth = [path_smooth, idx_start];
        idx_end = path_smooth(end);
        idx_start = path(end);
        idx_move = 1;
        
        disp(['changed--idx_start', num2str(idx_start)]);
        disp(['changed--idx_end', num2str(idx_end)]);
        disp('-------------------')
    end
end

end

