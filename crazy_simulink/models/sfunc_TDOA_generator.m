function [sys,x0,str,ts] = sfunc_TDOA_generator(t,x,u,flag,param)

switch flag,
    case 0
        [sys,x0,str,ts] = mdlInitializeSizes(param);
	case 2
        sys = mdlUpdates(x,u,param);
	case 3
        sys = mdlOutputs(x);
    case {1, 4, 9}
        sys = [];
    otherwise
        error(['Unhandled flag = ',num2str(flag)]);
end

function [sys,x0,str,ts] = mdlInitializeSizes(param)

sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = size(param.anchorPos,1);
sizes.NumOutputs     = size(param.anchorPos,1);
sizes.NumInputs      = 3;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes); 

x0 = zeros(size(param.anchorPos,1),1); % Initialises time lags as to zero
 
str = [];
ts  = [param.fRR 0]; % Initialise the round robin frequency, this can be
                     % to allow asynchronous updates of the changed times
                     % for a more realistic time difference generation.
                     
		      
function sys = mdlUpdates(x,u,param)
tP = u;                       % True position in the global frame
nA = size(param.anchorPos,1); % Number of anchors
c = param.c;                  % Speed of light
for ii = 1:nA
    x(ii) = norm(tP - param.anchorPos(ii,:)')/c;
end
sys = x;

%=========================================================================
% Calculate outputs - called on every simulation step, so the
% optimization whould be done in the update discrete states
%=========================================================================
function sys = mdlOutputs(x)
sys = x;
