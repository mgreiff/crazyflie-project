% Example showing how a 1D minimum snap trajectory can be computed through
% three points, where the positions x(t) are specified in all points, and
% the derivative v(t) is specified in the beginning and terminal point, in
% the middle point, the derivative is set free, trying to enforce
N = 4;
cost = [0, 0, 0, 0, 1];
times = [1, 2, 3, 4, 5, 6, 7];

x1 = [1; 0; NaN; NaN; NaN];
x2 = [1; inf; inf; NaN; NaN];
x3 = [0; inf; inf; NaN; NaN];
x4 = [1; inf; inf; NaN; NaN];
x5 = [2; inf; inf; NaN; NaN];
x6 = [1; inf; inf; NaN; NaN];
x7 = [0; 0; NaN; NaN; NaN];

Xpoints = [x1, x2, x3, x4, x5, x6, x7];

y1 = [0; 0; NaN; NaN; NaN];
y2 = [-1; inf; inf; NaN; NaN];
y3 = [0; inf; inf; NaN; NaN];
y4 = [1; inf; inf; NaN; NaN];
y5 = [3; inf; inf; NaN; NaN];
y6 = [3; inf; inf; NaN; NaN];
y7 = [1; 0; NaN; NaN; NaN];

Ypoints = [y1, y2, y3, y4, y5, y6, y7];

%% Sets up and solves the constrained QP-problem
PX = compute_splines( Xpoints , times , N , cost );
PY = compute_splines( Ypoints , times , N , cost );

%% Plots the results in a single graph
plot_splines_2D( PX , PY, Xpoints, Ypoints, times , {'position'})

title({'Positional trajectory in the xy-plane composed of',...
       'six splines at equidistant time intervals'})
xlabel('Time [s]')
ylabel('Position [m] and velocity [m/s]')
grid on;