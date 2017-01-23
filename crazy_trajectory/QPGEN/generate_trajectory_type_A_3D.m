% Example showing how a 3D minimum snap trajectory can be computed using
% compute_splines() and then evaluated using evaluate_splines()
N = 4;
cost = [0, 0, 0, 0, 1];
times = 2*[1, 2, 3, 4, 5, 6, 7, 8, 9]+3;

x1 = [1; 0; 0; NaN; NaN];
x2 = [0.5; inf; inf; NaN; NaN];
x3 = [1; inf; inf; NaN; NaN];
x4 = [1.5; inf; inf; NaN; NaN];
x5 = [1; inf; inf; NaN; NaN];
x6 = [0.5; inf; inf; NaN; NaN];
x7 = [1; inf; inf; NaN; NaN];
x8 = [1.5; inf; inf; NaN; NaN];
x9 = [1; 0; 0; NaN; NaN];

Xpoints = [x1, x2, x3, x4, x5, x6, x7, x8, x9];

y1 = [1; 0; 0; NaN; NaN];
y2 = [1.5; inf; inf; NaN; NaN];
y3 = [2; inf; inf; NaN; NaN];
y4 = [1.5; inf; inf; NaN; NaN];
y5 = [1; inf; inf; NaN; NaN];
y6 = [0.5; inf; inf; NaN; NaN];
y7 = [0; inf; inf; NaN; NaN];
y8 = [0.5; inf; inf; NaN; NaN];
y9 = [1; 0; 0; NaN; NaN];

Ypoints = [y1, y2, y3, y4, y5, y6, y7, y8, y9];

z1 = [1; 0; 0; NaN; NaN];
z2 = [1.25; inf; inf; NaN; NaN];
z3 = [1.3; inf; inf; NaN; NaN];
z4 = [1.25; inf; inf; NaN; NaN];
z5 = [1; inf; inf; NaN; NaN];
z6 = [1.25; inf; inf; NaN; NaN];
z7 = [1.3; inf; inf; NaN; NaN];
z8 = [1.25; inf; inf; NaN; NaN];
z9 = [1; 0; 0; NaN; NaN];
Zpoints = [z1, z2, z3, z4, z5, z6, z7, z8, z9];

%% Sets up and solves the constrained QP-problem
PX = compute_splines( Xpoints , times , N , cost );
PY = compute_splines( Ypoints , times , N , cost );
PZ = compute_splines( Zpoints , times , N , cost );

%% Plots the results in a single graph
plot_splines_3D({PX, PY, PZ}, {Xpoints, Ypoints, Zpoints}, times , {'position'})

title({'Three dimensional positional minimum snap trajectory [x(t), y(t), z(t)]^T'})
xlabel('x(t)'); xlabel('y(t)'); zlabel('z(t)')
grid on;

Pmat = [PX, PY, PZ, zeros(length(PX),1)];
save('trajTypeA', 'N', 'times', 'Pmat')