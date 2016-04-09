function [sys,x0,str,ts] = mpc_cvx_sfunc(t,x,u,flag,CVXparameters)

switch flag,
    case 0 % Initialization
        [sys,x0,str,ts] = mdlInitializeSizes(CVXparameters);
	case 2 % Update of discrete states
        sys = mdlUpdates(t,x,u,CVXparameters);
	case 3 % Calculation of outputs
        sys = mdlOutputs(t,x,u,CVXparameters);
    case {1, 4, 9}
        % 1 - Calculation of derivatives (not needed).
        % 4 - Calculation of next sample hit (variable sample time block only).
        % 9 - End of simulation tasks (not needed).
        sys = [];
    otherwise
        % No other flags are defined in simulink, throws error
        error(['Unhandled flag = ',num2str(flag)]);
end

function [sys,x0,str,ts] = mdlInitializeSizes(CVXparameters)

% Initialize simsizes
sizes = simsizes;

% No continuous states
sizes.NumContStates  = 0;

% AAll discrite states, both reshaped Rref and x 10 + 10*m in total
sizes.NumDiscStates  = CVXparameters.nDiscreteStates*...
                       (2+CVXparameters.predictionHorizon);

% Number of outputs (3)
sizes.NumOutputs     = CVXparameters.nOutputs;

% Number of inputs (1)
sizes.NumInputs      = CVXparameters.nInputs;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes); 

% Sets the initial condition in the reference trajectory to 0
x0 = [CVXparameters.x_0;...
      zeros(CVXparameters.nInputs-CVXparameters.nDiscreteStates,1)];
 
% Initialize the discrete states.
str = [];                        % Set str to an empty matrix.
ts  = [CVXparameters.h 0];       % sample time: [period, offset]
		      
%==============================================================
% Update the discrete states
%==============================================================
function sys = mdlUpdates(t,x,u,CVXparameters)

sys =  x;

%==============================================================
% Calculate outputs
%==============================================================
function sys = mdlOutputs(t,x,u,CVXparameters)

%fprintf('Update start, t=%4.2f\n',t)
nd = CVXparameters.nDiscreteStates;
np = CVXparameters.predictionHorizon;

% Updates the reference in the CVXparameters
Rref = reshape(u(nd+1:end),[nd,np+1]);
for ii = 0:10
    eval(['[CVXparameters.r_',num2str(ii),']=Rref(:,ii+1);'])
end

% Updates initial condition for states
CVXparameters.x_0 = u(1:nd);

% Sets u_0, has to be included properly in the solver first, currently at 0
% throughout

[vars, ~ ] = csolve(CVXparameters);

sys = vars.u_1;
