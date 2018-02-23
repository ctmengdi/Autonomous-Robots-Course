% input vertices
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

% convex
% vertices = [     0.1141 ,   0.5029,0;
%     0.2339  ,  0.7070,1;
%     0.3721  ,  0.5816,1;
%     0.3030  ,  0.4038,1;
%     0.4551  ,  0.3921,1;
%     0.5311  ,  0.6050,1;
%     0.4343  ,  0.7945,1;
%     0.8560  ,  0.5408,2;];


% % visualize the start and goal point
% figure;
% plot(vertices(1,1), vertices(1,2),'gx');
% hold on;
% plot(vertices(end,1), vertices(end,2),'gx');
% hold on;
% 
% % visualize the polygons
% for n = 1:size(edges_polyg,1)
%     plot([vertices(edges_polyg(n,1),1), vertices(edges_polyg(n,2),1)],...
%         [vertices(edges_polyg(n,1),2), vertices(edges_polyg(n,2),2)],'b');
%     hold on;
% end
% hold off;

tic
[ edges ] = RPS( vertices );
toc
%disp(edges);

tic
[path,minCost]=Astar(vertices, edges);
disp(minCost);
toc
plot_astar(vertices, edges, path);

% figure;
% for n = 1:size(edges,1)
%     plot([vertices(edges(n,1),1), vertices(edges(n,2),1)],...
%         [vertices(edges(n,1),2), vertices(edges(n,2),2)],'r');
%     hold on;
% end
% 
% 
% hold off;