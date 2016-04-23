function [sys,x0,str,ts] = AKF_sfunc(t,x,u,flag,param,trajectory)

switch flag,
    case 0 % Initialization
        [sys,x0,str,ts] = mdlInitializeSizes(param);
	case 2 % Update of discrete states
        sys = mdlUpdates(t,x,u,param,trajectory);
	case 3 % Calculation of outputs
        sys = mdlOutputs(t,x,u);
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
sizes.NumInputs      = param.nMeasuredStates;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes); 

x0 = param.x0;

str = [];                % Set str to an empty matrix.
ts  = [param.h 0];       % sample time: [period, offset]
		      
%==============================================================
% Update the discrete states
%==============================================================
function sys = mdlUpdates(t,x,u,param,trajectory)
Ad = param.Ad;
Cd = param.Cd;
Q = param.Q;
R = param.R;
h = param.h;

recN = round(param.recDelay/h);
sendN = round(param.sendDelay/h);

persistent P
persistent xhat
persistent posvec

if isempty(P) || t == 0
    P = param.P0;
    xhat = x;
    posvec = nan(1,recN);
end

posvec(2:end) = posvec(1:end-1);
posvec(1) = xhat(1);

Nx = param.nDiscreteStates;

zk = u(1:end);
   
% Predictor step
xf = Ad * xhat;
Pf = Ad * P * Ad' + Q;

% Corrector step
K =  Pf * Cd' / (Cd * Pf * Cd'+ R);
xhat = xf + K * (zk - Cd * xf);
P = (eye(Nx) - K * Cd) * Pf;

% Computes the number of steps to predict into the future
xout = xhat;
Ptemp = P;

aplitudeOffset = [xhat(1) - trajectory.pos(t);
                  0];

if sendN~=0
    for ii = 1:sendN
        % Predictor step
        xf = Ad * xout;
        Pf = Ad * Ptemp * Ad' + Q;
        zk = aplitudeOffset +[trajectory.pos(t+h*ii);trajectory.acc(t+h*ii)] + R*randn(2,1);

        % Corrector step
        K =  Pf * Cd' / (Cd * Pf * Cd'+ R);
        xout = xf + K * (zk - Cd * xf);
        Ptemp = (eye(Nx) - K * Cd) * Pf;
    end
end

sys = xout;

function sys = mdlOutputs(t,x,u)
% Returns the current estimation
sys = x;
