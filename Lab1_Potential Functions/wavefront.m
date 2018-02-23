function [value_map, trajectory]=wavefront(map, start_row, start_column, goal_row, goal_column)
%
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
tic

[row_num, col_num] = size(map);
pxl_value = 2;
map(goal_row, goal_column) = pxl_value;

while find(map(:,:)==0)
    for i = 1:row_num
        for j = 1:col_num
            
            neighbor8 = [i-1,j; i-1,j+1; i,j+1; i+1,j+1; ...
                         i+1,j; i+1,j-1; i,j-1; i-1,j-1];
                     
            if map(i,j) == pxl_value
                for n = 1:8
                    if neighbor8(n,1) <= 0 || neighbor8(n,1) > size(map,1) || ...
                            neighbor8(n,2) <= 0 || neighbor8(n,2) > size(map,2)
                        continue
                    end
                    
                    if  map(neighbor8(n,1),neighbor8(n,2))==0
                        map(neighbor8(n,1),neighbor8(n,2)) = pxl_value+1;
                    end
                end
            end
        end
    end
    pxl_value = pxl_value +1;
end

start_value = map(start_row, start_column);
trajectory = [start_value-1, 2];
trajectory(1,1) = start_row;
trajectory(1,2) = start_column;

for i = 1:start_value-2
    
    neighbor8 = [trajectory(i,1)-1,trajectory(i,2);
                trajectory(i,1)-1,trajectory(i,2)+1;
                trajectory(i,1),trajectory(i,2)+1;
                trajectory(i,1)+1,trajectory(i,2)+1;
                trajectory(i,1)+1,trajectory(i,2);
                trajectory(i,1)+1,trajectory(i,2)-1;
                trajectory(i,1),trajectory(i,2)-1;
                trajectory(i,1)-1,trajectory(i,2)-1];
                          
    for n = 1:8
        
        if neighbor8(n,1) <= 0 || neighbor8(n,1) > size(map,1) || ...
           neighbor8(n,2) <= 0 || neighbor8(n,2) > size(map,2)
        continue
        end
        
        if  map(neighbor8(n,1), neighbor8(n,2)) == ...
               map(trajectory(i,1), trajectory(i,2))-1
           
           if rem(n,2) == 1
               traj_n = n;
               break
           end
           if rem(n,2) == 0
               traj_n = n;
           end
           
        end
        
    end
        trajectory(i+1,1) = neighbor8(traj_n,1);
        trajectory(i+1,2) = neighbor8(traj_n,2);            
end
value_map = map;
toc
end

