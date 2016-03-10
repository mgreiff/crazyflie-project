function [sys,x0,str,ts] = MPC_CVX(t,x,u,flag,CVXparameters)

switch flag,
 case 0
  [sys,x0,str,ts] = mdlInitializeSizes(CVXparameters); % Initialization
 case 2
  sys = mdlUpdates(t,x,u,CVXparameters); % Update discrete states
 case 3
  sys = mdlOutputs(t,x,u,CVXparameters); % Calculate outputs
 case {1, 4, 9} % Unused flags
  sys = [];
  
 otherwise
  error(['unhandled flag = ',num2str(flag)]); % Error handling
end

function [sys,x0,str,ts] = mdlInitializeSizes(CVXparameters)
sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = CVXparameters.nDiscreteStates;
sizes.NumOutputs     = CVXparameters.nOutputs;
sizes.NumInputs      = CVXparameters.nInputs;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes); 
x0 = CVXparameters.x_0; % 10x1

global x;
x = x0;   
% Initialize the discrete states.
str = [];             % Set str to an empty matrix.
ts  = [CVXparameters.h 0];       % sample time: [period, offset]
% End of mdlInitializeSizes
		      
%==============================================================
% Update the discrete states
%==============================================================
function sys = mdlUpdates(t,x,u,CVXparameters)

sys =  x;
% End of mdlUpdate.

%==============================================================
% Calculate outputs
%==============================================================
function sys = mdlOutputs(t,x_i,u,CVXparameters)

global x;

fprintf('Update start, t=%4.2f\n',t)

% Optimize
%[duOpt,zOpt] = MPCOptimizeSolSC(x_hat,u_old,duOpt,refval,md);
[vars, ~ ] = csolve(CVXparameters);

xx = vars.x{:,1};

x = xx;

sys = [x];
