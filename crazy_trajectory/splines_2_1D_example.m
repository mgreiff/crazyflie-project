%% Sets order and declares cost
% The maximum order, N, of the splines ad the associated costs are in
% The example set to an integer 3,4,5; but any integer >1 is supported
% Given that the points ai have the correct number of elements (N+1)
% The cost matrix is set to minimum snap (minimizing jerk), but any
% combination of costs can be used
N = 4;
switch N
    case 3
        cost = [0,0,0,1];
        a1 = [5;NaN;NaN;NaN];
        a2 = [7;NaN;NaN;NaN];
        a3 = [0;NaN;NaN;NaN];
        a4 = [3;NaN;NaN;NaN];
    case 4
        cost = [0,0,0,1,0];
        a1 = [5;NaN;NaN;NaN;NaN];
        a2 = [7;NaN;NaN;NaN;NaN];
        a3 = [0;NaN;NaN;NaN;NaN];
        a4 = [3;NaN;NaN;NaN;NaN];
    case 5
        cost = [0,0,0,1,0,0]; 
        a1 = [5;NaN;NaN;NaN;NaN;NaN];
        a2 = [7;NaN;NaN;NaN;NaN;NaN];
        a3 = [0;NaN;NaN;NaN;NaN;NaN];
        a4 = [3;NaN;NaN;NaN;NaN;NaN];
end

%% Sets up the points and times
% Any number of splines can be used, but currently, the code only works
% with a single spline, the boundary conditions seem to be off when using
% more than one spline. In this example, the splines can be set to an
% an integer 1,2,3; but any number of splines could be used given
% consistent information on costs, times and points.

numberOfSplines = 3;
switch numberOfSplines
    case 1
        times = [5,10];
        points = [a1,a2];
    case 2
        times = [0,4,10];
        points = [a1,a2,a3];
    case 3
        times = [0,5,9,15];
        points = [a1,a2,a3,a4]; 
end

%% Sets up and solves the constrained QP-problem
P = compute_splines( points , times , N , cost );

%% Plots the results in a single graph
plot_splines( P , times , 'position')
plot_splines( P , times , 'velocity')
if N >= 2
    plot_splines( P , times , 'acceleration')
end
if N >= 3
    plot_splines( P , times , 'jerk')
end
grid on;