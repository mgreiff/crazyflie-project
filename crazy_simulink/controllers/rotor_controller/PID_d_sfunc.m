function [sys,x0,str,ts] = PID_d_sfunc(t,x,u,flag,param)

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
sizes.NumDiscStates  = 5;

% Number of outputs (3)
sizes.NumOutputs     = 1;

% Number of inputs (1)
sizes.NumInputs      = 2;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes); 

% Sets the initial condition in the reference trajectory to 0
x0 = [0; % u_k
      0; % r_k
      0; % y_k
      0; % I_k
      0];% D_k
 
% Initialize the discrete states.
str = [];                        % Set str to an empty matrix.
ts  = [param.h 0];       % sample time: [period, offset]
		      
function sys = mdlUpdates(t,x,u,param)
r_k = u(1);
y_k = u(2);
r_km1 = x(2);
y_km1 = x(3);
I_km1 = x(4);
D_km1 = x(5);

K = param.K;
Ti = param.Ti;
Td = param.Td;
N = param.N;
gamma = param.gamma;
beta = param.beta;
h = param.h;

P_k = K*(beta*r_k - y_k);
D_k = (Td / (Td + N * h)) * D_km1 - ((K * Td * N)/(Td + N * h)) *...
      (gamma * (r_k - r_km1) - (y_k - y_km1));
I_k = I_km1 + (K * h / Ti) * (r_k - y_k);

x(1) = P_k + I_k + D_k;
x(2) = r_k;
x(3) = y_k;
x(4) = I_k;
x(5) = D_k;
sys =  x;

%=========================================================================
% Calculate outputs - called on every simulation step, so the
% optimization whould be done in the update discrete states
%=========================================================================
function sys = mdlOutputs(x)
sys = x(1);
