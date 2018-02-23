% input map
map=[
1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1;
1 0 0 0 0 0 1 1 0 0 0 0 0 0 1 1 0 0 0 1;
1 0 0 0 0 0 1 1 0 0 0 0 0 0 1 1 0 0 0 1;
1 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 0 0 0 1;
1 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 0 0 0 1;
1 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 0 0 0 1;
1 0 0 0 1 1 1 1 1 0 0 0 0 0 1 1 0 0 0 1;
1 0 0 0 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 1;
1 0 0 0 0 0 0 1 1 0 0 0 0 0 0 0 0 0 0 1;
1 0 0 0 0 0 0 1 1 0 0 0 0 0 0 0 0 0 0 1;
1 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 1;
1 0 0 0 0 0 0 0 0 0 0 0 0 1 1 0 0 0 0 1;
1 0 0 0 0 0 0 0 0 0 0 0 1 1 1 0 0 0 0 1;
1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1;];

%input maze
% maze = load('maze.mat');
% map = maze.map;

%run brushfire algorithm
[value_map_b] = brushfire(map);
figure(1);
imagesc(value_map_b), colorbar;

%run wavefront algorithm
start_row = 45; start_column = 4;
goal_row = 5;   goal_column = 150;
[value_map_w, trajectory]=wavefront(map, start_row, start_column, goal_row, goal_column);

%visulize the trajectory
path_map = map;
for i = 1:size(map, 1)
    for j = 1:size(map, 2)
        for r = 1:size(trajectory, 1)
            if trajectory(r, 1) == i
                if trajectory(r, 2) == j
                    path_map(i, j) = 2;
                end
            end                           
        end
    end
end

path_map(goal_row, goal_column) = 3;
path_map(start_row, start_column) = 4;

figure(2);
I = imagesc(path_map); colorbar;


