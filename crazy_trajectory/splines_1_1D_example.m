% Given this problem setup, we expect the algorithm to find a spline
% s(t) which exists in t\in[0,T]=[0,5] where deg(s)=6 and
%
%    i) s(0) = 5, s(T) = 7
%    ii) s'(0) = -1, s'(T) = free
%    iii) s''(0) = s''(T) = free

% Points to pass through
a1 = [5; -1  ; NaN]; % pos, vel, acc, ...
a2 = [7; NaN; NaN];
points = [a1,a2]; 

% Predetermined times
times = [0,5];

% maximum order of the splines
N = 6;

% Minimum snap is ([0,0,0,1,0,0,0]) use other combinations for more aggressive solutions
cost = [0,0,0,1,0,0,0];
%cost = [0,0,1,0,0,0,0];
%cost = [10,0,1,0,0,0,0];

P = compute_splines( points , times , N , cost );

plot_splines( P , times , 'position')
plot_splines( P , times , 'velocity')
plot_splines( P , times , 'acceleration')