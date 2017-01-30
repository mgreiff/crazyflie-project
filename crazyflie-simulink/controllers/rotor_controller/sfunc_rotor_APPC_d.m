function [sys,x0,str,ts] = sfunc_rotor_PPC_d(t,x,u,flag,param)

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

sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 1;
sizes.NumOutputs     = 1;
sizes.NumInputs      = 6;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes); 

x0 = [0];
 
str = [];
ts  = [param.h 0];
		      
function sys = mdlUpdates(t,x,u,param)
uc_k = u(1);
y_k = u(2);

degT = param.degT;
degR = param.degR;
degS = param.degS;
ao = param.ao;
bm0 = param.bm0;
am1 = param.am1;
am2 = param.am2;

persistent uc_vec
persistent u_vec
persistent y_vec
if isempty(uc_vec) || isempty(u_vec) || isempty(y_vec)
    uc_vec = zeros(degT+1, 1);
    u_vec = zeros(degR+1, 1);
    y_vec = zeros(degS+1, 1);
end

y_vec = [y_k;y_vec(2:end)];
uc_vec = [uc_k;uc_vec(2:end)];

theta = u(3:end);
a1 = theta(1);
a2 = theta(2);
b0 = theta(3);
b1 = theta(4);

r1num = (b1^2 - am1 * b0 * b1 + am2 * b0^2) * (-b1 + ao * b0);
r1den = b0 * (b1^2 - a1 * b0 * b1 + a2 * b0^2);
r1 = b1/b0 + r1num/r1den;

s0numA = b1 * (ao * am1 - a2 - am1 * a1 + a1^2 + am2 - a1 * ao);
s0numB = b0 * (am1 * a2 -a1 * a2 - ao * am2 + ao * a2);
s0den = b1^2 - a1 * b0 * b1 + a2 * b0^2;
s0 = s0numA/s0den + s0numB/s0den;

s1numA = b1 * (a1 * a2 - am1 * a2 + ao * am2 - ao * a2);
s1numB = b0 * (a2 * am2 - a2^2 - ao * am2 * a1 + ao * a2 * am1);
s1den = b1^2 - a1 * b0 * b1 + a2 * b0^2;
s1 = s1numA/s1den + s1numB/s1den;

% Solves the diophantine equation to find the controller parameters
R = [1; r1];
S = [s0;s1];
T = ((1 + am1 + am2) / (b0 + b1)).* [1; ao];

if R(2:end)'*u_vec(2:end) ~= 0
    control = (T'*uc_vec - S'*y_vec) / R(2:end)'*u_vec(2:end);
else
    control = (T'*uc_vec - S'*y_vec);
end
u_vec = [control; u_vec(2:end)];
x = control;
sys = x;

%=========================================================================
% Calculate outputs - called on every simulation step, so the
% optimization whould be done in the update discrete states
%=========================================================================
function sys = mdlOutputs(x)
sys = x(1);
