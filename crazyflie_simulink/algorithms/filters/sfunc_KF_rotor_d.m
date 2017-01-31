function [sys,x0,str,ts] = sfunc_KF_rotor_d(t,x,u,flag,param)

switch flag,
    case 0 % Initialization
        [sys,x0,str,ts] = mdlInitializeSizes(param);
	case 2 % Update of discrete states
        sys = mdlUpdates(t,x,u,param);
	case 3 % Calculation of outputs
        sys = mdlOutputs(x);
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

% All discrite states, both reshaped Rref and x, 10 + (10+1)*m in total
sizes.NumDiscStates  = 2;

% Number of outputs (3)
sizes.NumOutputs     = 2;

% Number of inputs (1)
sizes.NumInputs      = 3;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes); 

% Sets the initial condition in the reference trajectory to 0
x0 = [0;  % w_0
      0];  % i_0

 
% Initialize the discrete states.
str = [];                        % Set str to an empty matrix.
ts  = [param.h 0];       % sample time: [period, offset]
		      
function sys = mdlUpdates(t,x,u,param)
if u(3) >= 0
   Ad = param.Adp; 
else
   Ad = param.Adm; 
end

Bd = param.Bd;
Cd = param.Cd;
Q = param.Q;
R = param.R;

persistent P
if isempty(P) || t == 0
    P = param.P0;
end

zk = u(1);
uk = u(2);

% Predictor step
xf = Ad * x + Bd * uk;
Pf = Ad * P * Ad' + Q;

% Corrector step
K =  Pf * Cd' / (Cd * Pf * Cd'+ R);
xhat = xf + K * (zk - Cd * xf);
P = (eye(2) - K * Cd) * Pf;

% Updates the states
sys = xhat;

%=========================================================================
% Calculate outputs - called on every simulation step, so the
% optimization whould be done in the update discrete states
%=========================================================================
function sys = mdlOutputs(x)
sys = x;
