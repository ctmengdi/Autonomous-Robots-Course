function [value_map] = brushfire(map) 
%BRUSHFIRE Summary of this function goes here
%   Detailed explanation goes here
tic

[row_num, col_num] = size(map);
pxl_value = 1;

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

value_map = map;
toc
end

