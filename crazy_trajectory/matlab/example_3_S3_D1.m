% Example showing how a 1D minimum snap trajectory can be computed through
% three points, where the positions x(t) are specified in all points, and
% the derivative v(t) is specified in the beginning and terminal point, in
% the middle point, the derivative is set free, trying to enforce
N = 4;
cost = [0,0,0,0,1];
a1 = [1; 0; NaN; NaN; NaN];
a2 = [2; inf; inf; NaN; NaN];
a3 = [-1; inf; inf; NaN; NaN];
a4 = [0; 1; NaN; NaN; NaN];

points = [a1, a2, a3, a4];
times = [0, 4, 8, 16]/4;

%% Sets up and solves the constrained QP-problem
P = compute_splines( points , times , N , cost );

%% Plots the results in a single graph
plot_splines( P , points, times , {'position', 'velocity'})
title({'Position and velocity as a function of time in a one dimensional',...
       'minimum snap trajectory composed of three splines'})
xlabel('Time [s]')
ylabel('Position [m] and velocity [m/s]')
grid on;