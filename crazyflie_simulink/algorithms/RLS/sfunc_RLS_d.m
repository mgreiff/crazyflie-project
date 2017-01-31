function [sys,x0,str,ts] = sfunc_RLS_d(t,x,u,flag,param)

switch flag,
    case 0
        [sys,x0,str,ts] = mdlInitializeSizes(param);
	case 2
        sys = mdlUpdates(t,x,u,param);
	case 3
        sys = mdlOutputs(x);
    case {1, 4, 9}
        sys = [];
    otherwise
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
sizes.NumOutputs     = 5;

% Number of inputs (1)
sizes.NumInputs      = 2;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes); 

% Raises exceptino if x0 is of the wrong dimensions and sets it to
% a zero vector of the correct dimensions if no x0 is provided.
nA = param.nA;
nB = param.nB;
try
    x0 = param.x0;
catch
    x0 = zeros(nA + nB,1); % Should be nA-1+nB, but we also need to
                           % initialize the state monitoring norm(P)
end
if length(x0) ~= (nA+nB)
     throw(MException('MYFUN:incorrectSize','Incorrect number of elements in x0.'))
end
if nB >= nA
     throw(MException('MYFUN:incorrectSize','The pulse transfer function must be proper.'))
end
str = [];
ts  = [param.h 0];
		      
function sys = mdlUpdates(t,x,u,param)
persistent P
persistent yvec
persistent uvec

lambda = param.lambda;
nA = param.nA - 1;
nB = param.nB;
delta = param.delta;

if isempty(P) || t == 0
    P = delta.*eye(nA+nB);
	yvec = zeros(nA,1);
	uvec = zeros(nB,1);
end
ynew = u(1);

estPar = x(1:nA+nB);
phi = [-yvec; uvec];

epsilon = ynew - phi'*estPar;
P = (1 / lambda) .* (P - P*(phi*phi')*P/(1 + phi'*P*phi));
K = P*phi;
estPar = estPar + K*epsilon;

yvec = [ynew; yvec(1:end-1)];
uvec = [u(2); uvec(1:end-1)];
sys = [estPar;norm(P)];

%=========================================================================
% Calculate outputs - called on every simulation step, so the
% optimization whould be done in the update discrete states
%=========================================================================
function sys = mdlOutputs(x)
sys = x;
