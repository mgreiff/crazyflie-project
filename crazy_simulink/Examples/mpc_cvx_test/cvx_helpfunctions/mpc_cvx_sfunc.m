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
%% Initialises the block where the form of the state vector is formed as
%
%     x = [u; x_hat; r_0; ...; r_m];
%
% where the dimensions of the control signal vector are
%
%     dim(u) = (3,1)
%
% and the dimensions of the measured states (column vector) are
%
%     dim(x_hat) = rim(r_i) = (numberOfStates, 1),
%
%     with dim(x) = (3 + numberOfStates * (2 + m), 1)

% Initialize simsizes
sizes = simsizes;

% No continuous states
sizes.NumContStates  = 0;

% All discrite states, both reshaped Rref and x, 10 + (10+1)*m in total
sizes.NumDiscStates  = 3 + CVXparameters.nDiscreteStates*...
                       (2+CVXparameters.predictionHorizon);

% Number of outputs (3)
sizes.NumOutputs     = CVXparameters.nOutputs;

% Number of inputs (1)
sizes.NumInputs      = CVXparameters.nInputs;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes); 

% Sets the initial condition in the reference trajectory to 0
x0 = [CVXparameters.u_0;... % initial u
      CVXparameters.x_0;... % initial x
      zeros(CVXparameters.nInputs-CVXparameters.nDiscreteStates,1)]; % dummy states
 
% Initialize the discrete states.
str = [];                        % Set str to an empty matrix.
ts  = [CVXparameters.h 0];       % sample time: [period, offset]
		      
%=========================================================================
% Update the discrete states - called once every CVXparameters.h seconds
%=========================================================================
function sys = mdlUpdates(t,x,u,CVXparameters)

nd = CVXparameters.nDiscreteStates;
np = CVXparameters.predictionHorizon;
h = CVXparameters.h;

% Updates the reference in the CVXparameters
Rref = reshape(u(nd+1:end),[nd,np+1]);
for ii = 0:10
    eval(['[CVXparameters.r_',num2str(ii),']=Rref(:,ii+1);'])
end

% Updates initial conditions for the optimization with new measurements
CVXparameters.x_0 = u(3+1:3+nd);

% Settings for the CVXgen solver
settings.verbose = CVXparameters.verbose;     % True by default
settings.max_iters = CVXparameters.max_iters; % 10 by default

if CVXparameters.verbose
    disp(['Current simulation time: ', num2str(t)])
end

[vars, ~ ] = csolve(CVXparameters, settings);

% Plots reference trajectory and optimized trajectory for debugging
if CVXparameters.plot
    Xval = [CVXparameters.x_0];
    for ii = 1:np
        eval(['Xval = [Xval, [vars.x_',num2str(ii),'()]];'])
    end
    figure(1);
    hold on;
    tt = t:h:t+h*np;
    plot(tt,Rref(1,:),'r')
    plot(tt,Rref(2,:),'b')
    plot(tt,Rref(3,:),'g')
    plot(tt,Xval(1,:),'r*')
    plot(tt,Xval(2,:),'b*')
    plot(tt,Xval(3,:),'g*')
end

x(1:3) = vars.u_1;
sys =  x;

%=========================================================================
% Calculate outputs - called on every simulation step, so the
% optimization whould be done in the update discrete states
%=========================================================================
function sys = mdlOutputs(t,x,u,CVXparameters)
out = x(1:3);
sys = out;
