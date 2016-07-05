 P1 = [0,0];
P2 = [5,2];
hold on;
plot([P1(1), P2(1)], [P1(2), P2(2)], 'k-*')
text(P1(1), P1(2), 'P1')
text(P2(1), P2(2), 'P2')
axis equal

pause;
% Obstacle 1
x1 = rand(40,1) + 3;
y1 = rand(40,1) + 1;
plot(x1,y1, '*r');

% Obstacle 2
x2 = rand(40,1)+1;
y2 = rand(40,1);
plot(x2,y2, '*g');

pause;
cla;
plot([P1(1), P2(1)], [P1(2), P2(2)], 'k-*')
text(P1(1), P1(2), 'P1')
text(P2(1), P2(2), 'P2')
k = convhull(x1,y1);
plot(x1(k), y1(k),'-*r')
k = convhull(x2,y2);
plot(x2(k), y2(k),'-*g')

pause;
cla;
plot([P1(1), P2(1)], [P1(2), P2(2)], 'k-*')
text(P1(1), P1(2), 'P1')
text(P2(1), P2(2), 'P2')

d = 0.2; % Radius

[x1, y1] = cs_obstacle(x1, y1, d);
plot(x1, y1, '-*r')

[x2, y2] = cs_obstacle(x2, y2, d);
plot(x2, y2, '-*g')

pause;
cla;
plot([P1(1), P2(1)], [P1(2), P2(2)], 'k-*')
text(P1(1), P1(2), 'P1')
text(P2(1), P2(2), 'P2')

plot(x1, y1, '-*r')
for ii = 1:length(x1)
    rectangle('Position',[x1(ii)-d,y1(ii)-d, 2 * d, 2 * d],'Curvature',[1 1])
end

plot(x2, y2, '-*g')
for ii = 1:length(x2)
    rectangle('Position',[x2(ii)-d,y2(ii)-d, 2 * d, 2 * d],'Curvature',[1 1])
end

pause;
%% Finds the first intersecting edge
XY = [x1(1:end-1)', y1(1:end-1)', x1(2:end)',  y1(2:end)'];
tic
out = lineSegmentIntersect([P1(1),P1(2),P2(1),P2(2)],XY);
disp(['Computed all intersections of the ', num2str(length(x1)),...
      ' line segments in ', num2str(toc), 'seconds.'])
index = find(out.intAdjacencyMatrix > 0);
maxdist = 0;
maxind = 0;
for ii = index
    dist = ((P1(1) - x1(ii)).^2 + (P1(2) - y1(ii)).^2);
    if dist > maxdist
        maxind = ii;
        maxdist = dist;
    end
end
dir = [x1(maxind+1) - x1(maxind), y1(maxind+1) - y1(maxind)];
dir = 2*dir./norm(dir);
plot([x1(maxind), dir(1) + x1(maxind)], [y1(maxind), dir(2) + y1(maxind)], 'k--')
plot([x1(maxind), -dir(1) + x1(maxind)], [y1(maxind), -dir(2) + y1(maxind)], 'k--')
