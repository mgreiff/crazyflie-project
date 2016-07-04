%% Sets order and declares cost
N = 4;
cost = [0,0,0,0,1];
a1 = [1;-1;NaN;NaN;NaN];
a2 = [2; 0; NaN;NaN;NaN];
a3 = [0; -1;NaN;NaN;NaN];
a4 = [3; 0;NaN;NaN;NaN];


points = [a1, a2, a3, a4];
times = [0, 4, 8, 12];
points  = [a1, a2, a3, a4];
times  = [0, 4, 8, 12];

%% Sets up and solves the constrained QP-problem
P = compute_splines( points , times , N , cost );

%% Plots the results in a single graph
plot_splines( P , times , {'position', 'velocity', 'acceleration', 'jerk'})
grid on;