%% Sets order and declares cost
N = 4;
cost = [0,0,0,0,1];
a1 = [1;-1;NaN;NaN;NaN];
a2 = [2; 0;NaN;NaN;NaN];

points = [a1, a2];
times = [0,4];

%% Sets up and solves the constrained QP-problem
P = compute_splines( points , times , N , cost );

%% Plots the results in a single graph
plot_splines( P , points, times, {'position', 'velocity'})
grid on;