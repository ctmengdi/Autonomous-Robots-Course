function plot_astar(vertices, edges, path)
figure;hold on;
% First plot all the available route
for i = 1 : size(edges,1)
    plot([vertices(edges(i,1),1),vertices(edges(i,2),1)],[vertices(edges(i,1),2),vertices(edges(i,2),2)],'r');
end
% Second plot the vertices
for i = 1 : size(vertices,1)
    plot(vertices(i,1),vertices(i,2),'bo','MarkerSize', 2);
    text(vertices(i,1),vertices(i,2),num2str(i),'HorizontalAlignment','left','FontSize',10);
end
% Third plot the polygons
for i = 2 : size(vertices,1) - 1
    if vertices(i + 1,3) ~= vertices(i,3) 
        num_v = sum(vertices(:,3) == vertices(i,3)); % Number of vertices in the current polygon
        plot([vertices(i,1),vertices(i-num_v+1,1)],[vertices(i,2),vertices(i-num_v+1,2)],'b-','LineWidth',1);
    else
        plot([vertices(i,1),vertices(i+1,1)],[vertices(i,2),vertices(i+1,2)],'b-','LineWidth',1);
    end
end
% Finally plot the path
for i = 1 : size(path,2)-1
    plot([vertices(path(i),1),vertices(path(i + 1),1)],[vertices(path(i),2), vertices(path(i + 1), 2)],'g-','LineWidth',1.5);
end
hold off;
end