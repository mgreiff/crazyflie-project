%% Set up PD augmented linearized model
Ad = diag(A);
a1 = 4.8; a2 = 8.32; b = 8.32;
a1 = KetaD(1); a2 = KetaP(1); b = KetaP(1);

T0 = m*g;

Am1 = [zeros(10,3)];
Am2 = [eye(3); -Ad/m; zeros(4,3)];
Am3 = [zeros(3,2); T0/m 0; 0 -T0/m; zeros(3,2); -a2*eye(2)];
Am4 = [zeros(6,2); eye(2); -a1*eye(2)];
Am = [Am1 Am2 Am3 Am4];

Bm = zeros(10, 3);
Bm(9, 1) = b;
Bm(10, 2) = b;
Bm(6, 3) = 1/m;

Cm = zeros(5, 10);
Cm(1:3, 1:3) = eye(3);
Cm(4:5, 7:8) = eye(2);

Dm = zeros(5, 3);

%% Discretize
Ts = 1; % sample time

[Ad, Bd, Cyd, Dzd]=ssdata(c2d(ss(Am,Bm,Cm,Dm),Ts));

disp('Generate CVXGEN data struct...')
% Must be specified to run the solver eg using
% 
%   [vars, status] = csolve(CVXparameters)
%

% Dynamics and initial conditions
CVXparameters.A = Ad;
CVXparameters.B = Bd;
CVXparameters.x_0 = [1;1;1;0;0;0;0;0;0;0];

% Design parameters
CVXparameters.Q = eye(10);
CVXparameters.R = eye(3);
CVXparameters.S_max = 1000;
CVXparameters.u_max = 10000;

% Must be specified to make the S-fucntion run
CVXparameters.nDiscreteStates = 10; %?
CVXparameters.nOutputs = 10; %?
CVXparameters.nInputs = 10; %?
CVXparameters.h = Ts; % Stepsize
disp('...complete!')

%% This example simply executes one MPC cycle with some referece curve,
%  where positions change in time (on the prediction horizon) but the
%  speeds and angular parameters are kept constant.

% Specifies a trajecotry and plots the response
CVXparameters.r_0 = [1.0,1.00,1.0,0,0,0,0,0,0,0]';
CVXparameters.r_1 = [1.1,0.90,1.0,0,0,0,0,0,0,0]';
CVXparameters.r_2 = [1.2,0.81,1.1,0,0,0,0,0,0,0]';
CVXparameters.r_3 = [1.3,0.73,1.1,0,0,0,0,0,0,0]';
CVXparameters.r_4 = [1.4,0.65,1.2,0,0,0,0,0,0,0]';
CVXparameters.r_5 = [1.5,0.55,1.2,0,0,0,0,0,0,0]';
CVXparameters.r_6 = [1.6,0.49,1.2,0,0,0,0,0,0,0]';
CVXparameters.r_7 = [1.7,0.42,1.1,0,0,0,0,0,0,0]';
CVXparameters.r_8 = [1.8,0.39,1.1,0,0,0,0,0,0,0]';
CVXparameters.r_9 = [1.9,0.38,1.0,0,0,0,0,0,0,0]';
CVXparameters.r_10 = [2.0,0.38,1.0,0,0,0,0,0,0,0]';

tt = Ts.*(0:10); % Times on the control horizon, with t_0 = 0

optsol = csolve(CVXparameters); % Solves problem

Rref = []; % Retrievs the reference trajectories defined above
for ii = 0:10
    cmd = ['Rref=[Rref,CVXparameters.r_', num2str(ii),'];'];
    eval(cmd)
end
Xval = [CVXparameters.x_0]; % Retrievs the optimal states
for ii = 1:10
    cmd = ['Xval=[Xval,optsol.x_', num2str(ii),'];'];
    eval(cmd)
end
Uval = []; % Retrievs the optimal control signal
for ii = 1:10
    cmd = ['Uval=[Uval,optsol.u_', num2str(ii),'];'];
    eval(cmd)
end

% Plots states and references (prediction horizon)
figure(1);
subplot(221)
hold on;
plot(tt,Rref(1:3,1:11))
plot(tt,Xval(1:3,1:11))
legend('x_{ref}','y_{ref}','z_{ref}','x','y','z')
title(['Reference trajectory \bf R_{ref}, and the resulting states as solved',...
       'using CVXgen for a single MPC cycle'])
   
subplot(222)
hold on;
plot(tt,Rref(4:6,1:11))
plot(tt,Xval(4:6,1:11))
legend('dx_{ref}','dy_{ref}','dz_{ref}','dx','dy','dz')

subplot(223)
hold on;
plot(tt,Rref(7:8,1:11))
plot(tt,Xval(7:8,1:11))

subplot(224)
hold on;
plot(tt,Rref(9:10,1:11))
plot(tt,Xval(9:10,1:11))

% Plots control signals
figure(2);
plot(tt(2:end),Uval)
title(['Optimal control signal, solved using CVXgen for a single MPC cycle'])