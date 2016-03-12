% Points to pass through
a1 = [5; NaN; NaN]; % pos, vel, acc, ...
a2 = [7; NaN; NaN];
a3 = [3; NaN; NaN];
points = [a1,a2,a3]; 

% Predetermined times
times = [0,1,2];

% maximum order of the splines
N = 8;

% Minimum snap is ([0,0,0,1,0,0,0]) use other combinations for more aggressive solutions
cost = [0,0,0,1,0,0,0,0,0];
%cost = [0,0,1,0,0,0,0];
%cost = [10,0,1,0,0,0,0];

P = compute_splines( points , times , N , cost );

plot_splines( P , times , 'position')
plot_splines( P , times , 'velocity')
plot_splines( P , times , 'acceleration')