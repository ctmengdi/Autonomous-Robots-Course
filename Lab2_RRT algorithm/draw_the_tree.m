% Load the map
map = load('map.mat');  % load the map
map = map.map;
% maze = load('maze.mat');  % load the maze
% map = maze.map;

% Parameter initialization
q_start = [80, 70];   % start and goal for map
q_goal = [707, 615];
% q_start = [206, 198];   % start and goal for maze
% q_goal = [416, 612];

k = 10000;
delta_q = 50;
p = 0.3;

% Call the rrt function
[vertices,edges,path]=rrt(map,q_start,q_goal,k,delta_q,p);

% Show the map
imshow(1-map,[]);
hold on

% Plot the vertices
plot(vertices(:,1), vertices(:,2), '*g');
hold on

% Plot the start and goal points
scatter(q_start(1), q_start(2), 'or', 'filled');
hold on
scatter(q_goal(1), q_goal(2), 'or', 'filled');
hold on

% Plot the edges
for i = 1:size(edges,1)
    line_start = edges(i,1); line_end = edges(i,2);
    plot([vertices(line_end, 1) vertices(line_start,1)],...
        [vertices(line_end, 2) vertices(line_start,2)], '-b');
    hold on
end

% Plot the path
for i = 1:size(path,2)-1
    idx_vertex = path(i);
    idx_next_vertex = path(i+1);
    plot([vertices(idx_vertex, 1) vertices(idx_next_vertex,1)],...
        [vertices(idx_vertex, 2) vertices(idx_next_vertex,2)], '-r');
    hold on
end


delta = 5;
[path_smooth]=smooth(map,path,vertices,delta);

% Plot the path_smooth
for i = 1:size(path_smooth,2)-1
    idx_vertex = path_smooth(i);
    idx_next_vertex = path_smooth(i+1);
    plot([vertices(idx_vertex, 1) vertices(idx_next_vertex,1)],...
        [vertices(idx_vertex, 2) vertices(idx_next_vertex,2)], '-k');
    hold on
end

hold off
