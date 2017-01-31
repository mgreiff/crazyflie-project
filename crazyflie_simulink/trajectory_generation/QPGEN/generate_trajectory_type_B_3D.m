% Example showing how a 3D minimum snap trajectory can be computed using
% compute_splines() and then evaluated using evaluate_splines()
N = 4;
cost = [0, 0, 0, 0, 1];
startTime = 2;
%splineTimes = 0.1.*[3, 1, 1, 2, 2, 1, 1, 3];
splineTimes = 0.2.*[2, 1, 1, 1, 1, 1, 1, 3];
times = cumsum([startTime, splineTimes]);
loopFactor = 0.2;

x0 = [0; 0; 0; NaN; NaN];
x1 = [1; inf; inf; NaN; NaN];
x2 = [1.5946; inf; inf; NaN; NaN];
%x3 = [1.7071; inf; inf; NaN; NaN];
x4 = [1.5946; inf; inf; NaN; NaN];
x5 = [1; inf; inf; NaN; NaN];
x6 = [0.4054; inf; inf; NaN; NaN];
%x7 = [0.2929; inf; inf; NaN; NaN];
x8 = [0.4054; inf; inf; NaN; NaN];
x9 = [1; inf; inf; NaN; NaN];
x10 = [2; 0; 0; NaN; NaN];

Xpoints = [x0, x1, x2, x4, x5, x6, x8, x9, x10];

y0 = [0; 0; 0; NaN; NaN];
y1 = [1; inf; inf; NaN; NaN];
y2 = [1.5946; inf; inf; NaN; NaN];
%y3 = [1.7071; inf; inf; NaN; NaN];
y4 = [1.5946; inf; inf; NaN; NaN];
y5 = [1; inf; inf; NaN; NaN];
y6 = [0.4054; inf; inf; NaN; NaN];
%y7 = [0.2929; inf; inf; NaN; NaN];
y8 = [0.4054; inf; inf; NaN; NaN];
y9 = [1; inf; inf; NaN; NaN];
y10 = [2; 0; 0; NaN; NaN];

Ypoints = [y0, y1, y2, y4, y5, y6, y8, y9, y10];

z0 = [0; 0; 0; NaN; NaN];
z1 = [0; inf; inf; NaN; NaN];
z2 = [0.25; inf; inf; NaN; NaN];
%z3 = [1.5; inf; inf; NaN; NaN];
z4 = [0.75; inf; inf; NaN; NaN];
z5 = [1; inf; inf; NaN; NaN];
z6 = [0.75; inf; inf; NaN; NaN];
%z7 = [1.5; inf; inf; NaN; NaN];
z8 = [0.25; inf; inf; NaN; NaN];
z9 = [0; inf; inf; NaN; NaN];
z10 = [0; 0; 0; NaN; NaN];

Zpoints = [z0, z1, z2, z4, z5, z6, z8, z9, z10];

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
save('trajTypeB', 'N', 'times', 'Pmat')