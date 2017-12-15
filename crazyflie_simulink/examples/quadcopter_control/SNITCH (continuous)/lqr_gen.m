clear all; close all; clc;
% -------------- Problem Definition --------------
% System parameters
%   timestep
Ts = 0.001;

%   quadrotor
m = 0.03097;
Jx = 1.112951e-5;
Jy = 1.1143608e-5;
Jz = 2.162056e-5;
I = diag([Jx Jy Jz]);
L = 0.0465;

% Organize
quad_params = [m, Jx, L];

% -------------- System equations --------------
fprintf("Generating analytical system dynamics...\n");
[f, q, q_dot, u] = quad_eqngen(quad_params);
%vpa(f,4)

% separate position and attitude dynamics
f_p = [f(1:3); f(7:9)];
f_r = [f(4:6); f(10:12)];

% -------------- Linearization & LQR --------------
ep = 1e-3;
% ------ Simple LQR
fprintf("Computing simple LQR gains...\n");
% Optimization parameters
%Q = diag([1 1 1 ep ep 1 ep ep ep ep ep ep]);
Q = diag([ep ep ep 1 1 1 ep ep ep ep ep ep]);
R = diag([1 1 1 1]);

% LQR around equilibrium
x_l = [q.', q_dot.'];
u_l = u.';
x_l0 = zeros(1,12);
u_l0 = zeros(1,4);
x_l0(4) = 0; x_l0(5) = 0; 
u_l0(1) = solve(subs(f(9), x_l(4:6).', x_l0(4:6).'), u(1));

[A, B] = linearize(f, x_l, u_l, x_l0, u_l0);
K_lqr = lqrd(A, B, Q, R, Ts);

% Save precomputed data to file
save('lqr_resource.mat', 'quad_params', 'K_lqr', 'x_l0', 'u_l0');

% ------ Two-Part LQR
fprintf("Computing two-part LQR gains...\n");
% Optimization parameters
Q_p = 1e2*diag([1 1 1 ep ep ep]);
R_p = 1e2*diag([10 1 1 1]);
Q_r = diag([10 10 2 1 1 0.2]);
R_r = 5e5*diag([1 1 1]);

% LQR around equilibrium
x_lp = [q(1:3).', q_dot(1:3).'];
u_lp = [u(1), q(4:6).'];
x_lp0 = zeros(1,6);
u_lp0 = zeros(1,4);
x_lp0(4) = 0; x_lp0(5) = 0; 
u_lp0(1) = solve(subs(f(9), x_l(4:6).', x_l0(4:6).'), u(1));

[A_p, B_p] = linearize(f_p, x_lp, u_lp, x_lp0, u_lp0);
K_lqr_p = lqrd(A_p, B_p, Q_p, R_p, Ts);

x_lr = [q(4:6).', q_dot(4:6).'];
u_lr = u(2:4).';
x_lr0 = zeros(1,6);
u_lr0 = zeros(1,3);
x_lr0(4) = 0; x_lr0(5) = 0;

[A_r, B_r] = linearize(f_r, x_lr, u_lr, x_lr0, u_lr0);
K_lqr_r = lqrd(A_r, B_r, Q_r, R_r, Ts);
vpa(K_lqr_r, 7)

save('tplqr_resources', 'quad_params', 'K_lqr_p', 'K_lqr_r', 'x_lp0', 'x_lr0', 'u_lp0', 'u_lr0')

%{
% ------ Gain Scheduling LQR
fprintf("Computing GS LQR gains...\n");
% Initialize
lqr_database = {};
x_l = [q.', q_dot.'];
u_l = u.';

% Iterate through all operating points
i = 0; j = 0; k = 0;
for q4_ls = linspace(-1,1,5)
    i = i + 1;
    for q5_ls = linspace(-1,1,5)
        j = j + 1;
        for u1_ls = linspace(0,2,3)*subs(f(9), u(1), 0)
            k = k + 1;
            % linearize around operation point
            x_l0 = zeros(1,12);
            u_l0 = zeros(1,4);
            x_l0(4) = q4_ls;
            x_l0(5) = q5_ls;
            u_l0(1) = u1_ls;

            [A, B] = linearize(f, x_l, u_l, x_l0, u_l0);
            K = lqr(A, B, Q, R);
            
            % save to database
            lqr_database{i,j,k} = {x_l0, u_l0, A, B, K};
            fprintf(".");
        end
    end
end

% save precomputed data to file
save('parameters.mat', 'quad_params', 'motor_params', 'v2u', 'u2v');
save('gslqr_database.mat', 'lqr_database', 'q4_ls', 'q5_ls', 'u1_ls');
%}
