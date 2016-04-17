function [sys,x0,str,ts] = KF_sfunc(t,x,u,flag,param)

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
global P;
P = param.P0;

str = [];                % Set str to an empty matrix.
ts  = [param.h 0];       % sample time: [period, offset]
		      
%==============================================================
% Update the discrete states
%==============================================================
function sys = mdlUpdates(t,x,u,param)
Ad = param.Ad;
Bd = param.Bd;
Cd = param.Cd;
Q = param.Q;
R = param.R;

persistent P
if isempty(P)
    P = param.P0;
end

uk = u(1:4);
zk = u(5:end);

% Predictor step
xf = Ad * x + Bd * uk;
Pf = Ad * P * Ad' + Q;

% Corrector step
K =  Pf * Cd' / (Cd * Pf * Cd'+ R);
xhat = xf + K * (zk - Cd * xf);
P = (eye(12) - K * Cd) * Pf;

% Updates the states
sys = xhat;

function sys = mdlOutputs(t,x,u,param)
% Returns the current estimation
sys = x;
