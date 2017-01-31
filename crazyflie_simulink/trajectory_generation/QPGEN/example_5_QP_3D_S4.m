% Example showing how a 3D minimum snap trajectory can be computed using
% compute_splines() and then evaluated using evaluate_splines()
N = 4;
cost = [0, 0, 0, 0, 1];
times = [1, 2, 3, 4, 5];

x1 = [0; 0; NaN; NaN; NaN];
x2 = [1; inf; inf; NaN; NaN];
x3 = [0; inf; inf; NaN; NaN];
x4 = [-2; inf; inf; NaN; NaN];
x5 = [0; 0; NaN; NaN; NaN];

Xpoints = [x1, x2, x3, x4, x5];

y1 = [-1; 0; NaN; NaN; NaN];
y2 = [-1; inf; inf; NaN; NaN];
y3 = [0; inf; inf; NaN; NaN];
y4 = [1; inf; inf; NaN; NaN];
y5 = [1; 0; NaN; NaN; NaN];

Ypoints = [y1, y2, y3, y4, y5];

z1 = [0; 0; NaN; NaN; NaN];
z2 = [2; inf; inf; NaN; NaN];
z3 = [3; inf; inf; NaN; NaN];
z4 = [2; inf; inf; NaN; NaN];
z5 = [1; 0; NaN; NaN; NaN];

Zpoints = [z1, z2, z3, z4, z5];

%% Sets up and solves the constrained QP-problem
PX = compute_splines( Xpoints , times , N , cost );
PY = compute_splines( Ypoints , times , N , cost );
PZ = compute_splines( Zpoints , times , N , cost );

%% Plots the results in a single graph
plot_splines_3D({PX, PY, PZ}, {Xpoints, Ypoints, Zpoints}, times , {'position'})

title({'Three dimensional positional minimum snap trajectory [x(t), y(t), z(t)]^T'})
xlabel('x(t)'); xlabel('y(t)'); zlabel('z(t)')
grid on;