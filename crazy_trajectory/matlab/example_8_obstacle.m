% Example showing how a 3D minimum snap trajectory can be computed using
% compute_splines() and then evaluated using evaluate_splines()
N = 4;
cost = [0, 0, 0, 0, 1];
times = [1, 2, 3, 4];

x1 = [0; 1; NaN; NaN; NaN];
x2 = [2; 1; inf; NaN; NaN];
x3 = [3; 1; inf; NaN; NaN];
x4 = [4; 1; NaN; NaN; NaN];

Xpoints = [x1, x2, x3, x4];

y1 = [0; 1/3; NaN; NaN; NaN];
y2 = [2; 1; inf; NaN; NaN];
y3 = [2; -1; inf; NaN; NaN];
y4 = [1; 1/3; NaN; NaN; NaN];

Ypoints = [y1, y2, y3, y4];

%% Sets up and solves the constrained QP-problem
PX = compute_splines( Xpoints , times , N , cost );
PY = compute_splines( Ypoints , times , N , cost );

% Plot obstacle
hold on;
plot([0,4], [0,1], 'b--')
axis equal 
xlabel('x(t)'); xlabel('y(t)');
pause;
d = 0.2;
delta = 0.03;
plot([2.2-d/2-delta,2.2-d/2-delta,3-d/2+delta,3-d/2+delta],...
     [-.5,2-d/2+delta,2-d/2+delta,-.5], 'k')
pause;
rectangle('Position',[2.2-d-delta,2-d+delta, d, d],'Curvature',[1 1])
rectangle('Position',[3-d+delta,2-d+delta, d, d],'Curvature',[1 1])
pause;
cla;
plot([2.2-d/2-delta,2.2-d/2-delta,3-d/2+delta,3-d/2+delta],...
     [-.5,2-d/2+delta,2-d/2+delta,-.5], 'k')
plot([0,2+delta,3-delta,4], [0,2+delta,2+delta,1], 'b--')
plot([2.2-d/2-delta,2.2-d/2-delta,3-d/2+delta,3-d/2+delta],...
     [-.5,2-d/2+delta,2-d/2+delta,-.5], 'k')
rectangle('Position',[2.2-d-delta,2-d+delta, d, d],'Curvature',[1 1])
rectangle('Position',[3-d+delta,2-d+delta, d, d],'Curvature',[1 1])
pause;
%% Plots the results in a single graph
plot_splines_3D({PX, PY}, {Xpoints, Ypoints}, times , {'position'})