function [sys,x0,str,ts] = sfunc_inner_APPC_thrust_d(t,x,u,flag)

switch flag,
    case 0
        [sys,x0,str,ts] = mdlInitializeSizes();
	case 2
        sys = mdlUpdates(t,x,u);
	case 3
        sys = mdlOutputs(x);
    case {1, 4, 9}
        sys = [];
    otherwise
        error(['Unhandled flag = ',num2str(flag)]);
end

function [sys,x0,str,ts] = mdlInitializeSizes()

sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 2;
sizes.NumOutputs     = 1;
sizes.NumInputs      = 6;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes); 

x0 = [0;0];
 
str = [];
ts  = [0.002 0];
		      
function sys = mdlUpdates(t,x,u)
z_ref = u(1);
z_hat = u(2);
a_hat = u(3:4);
b_hat = u(5:6);
u_km1 = x(1);
z_km1 = x(2);

% Design specifications
b1m = 3.996e-06;
a1m = -1.997;
a2m = 0.9972;

a1 = a_hat(1);
a2 = a_hat(2);
b1 = b_hat(1);
b2 = b_hat(2);

% Solves the diophantine equation to find the controller parameters
r1 = b1/b1;
s0 = (a1m - a1)/b1;
s1 = (a2m-a2)/b1;
t0 = b1m/b1;

%x(1) = -r1*u_km1 - s0 * z_hat - s1 * z_km1 + t0 * z_ref;
x(2) = z_hat;
sys = x;

%=========================================================================
% Calculate outputs - called on every simulation step, so the
% optimization whould be done in the update discrete states
%=========================================================================
function sys = mdlOutputs(x)
sys = x(1);
