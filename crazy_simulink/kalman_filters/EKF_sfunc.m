function [sys,x0,str,ts] = EKF_sfunc(t,x,u,flag,param)

switch flag,
    case 0 % Initialization
        [sys,x0,str,ts] = mdlInitializeSizes(param);
	case 2 % Update of discrete states
        sys = mdlUpdates(t,x,u,param);
	case 3 % Calculation of outputs
        sys = mdlOutputs(t,x,u,param);
    case {1, 4, 9}
        % 1 - Calculation of derivatives (not needed).
        % 4 - Calculation of next sample hit (variable sample time block only).
        % 9 - End of simulation tasks (not needed).
        sys = [];
    otherwise
        % No other flags are defined in simulink, throws error
        error(['Unhandled flag = ',num2str(flag)]);
end

function [sys,x0,str,ts] = mdlInitializeSizes(param)

% Initialize simsizes
sizes = simsizes;

% No continuous states
sizes.NumContStates  = 0;

% AAll discrite states, both reshaped Rref and x 10 + 10*m in total
sizes.NumDiscStates  = param.nDiscreteStates;

% Number of outputs (3)
sizes.NumOutputs     = param.nOutputs;

% Number of inputs (1)
sizes.NumInputs      = param.nInputs;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes); 

x0 = param.x0;
str = [];                % Set str to an empty matrix.
ts  = [param.h 0];       % sample time: [period, offset]
		      
%==============================================================
% Update the discrete states
%==============================================================
function sys = mdlUpdates(t,x,u,param)

persistent P
if isempty(P)
    P = param.P0;
elseif t == 0
    P = param.P0;
end

states = x;

x = states(1);
y  = states(2);
z = states(3);
xd = states(4);
yd = states(5);
zd = states(6);
psi = states(7);
theta  = states(8);
phi = states(9);
phidot = states(10);
thetadot = states(11);
psidot = states(12);

Jh = eval(param.EKFLinSys);

uk = u(1:4);
zk = u(5:end);
Cd = param.Cd;
Q = param.Q;
R = param.R;

% Predictor step
xf = discrete_nonlinear_dynamics(uk, states, param.g, param.m, param.k,...
                                 param.A, param.I,param.l, param.b,...
                                 param.h);
Pf = Jh * P * Jh' + Q;

% Corrector step
K =  Pf * Cd' / (Cd * Pf * Cd'+ R);
xhat = xf + K * (zk - Cd * xf);
P = (eye(12) - K * Cd) * Pf;
sys = xhat;

%==============================================================
% Calculate outputs
%==============================================================
function sys = mdlOutputs(t,x,u,param)

sys = x;
