function [sys,x0,str,ts] = sfunc_inner_PID_position_d(t,x,u,flag,param)

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
sizes.NumDiscStates  = 8;
sizes.NumOutputs     = 4;
sizes.NumInputs      = 24;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes); 

x0 = [0;...  % phi_ref
      0;...  % theta_ref
      0;...  % e_x
      0;...  % e_y
      0;...  % I_x
      0;...  % I_y
      0;...  % D_x
      0];    % D_y
 
str = [];
ts  = [param.h 0];
		      
function sys = mdlUpdates(t,x,u,param)
ref = u(1:12);
x_hat = u(13:24);

% Translational control error in the global coordinates in the xy-plane
eG = [ref(1) - x_hat(1);...
      ref(2) - x_hat(2);...
      ref(3) - x_hat(3)];

% Compute Rotation matrix
phi = x_hat(7);
theta = x_hat(8);
psi = x_hat(9);
Sphi = sin(phi); Stheta = sin(theta); Spsi = sin(psi);
Cphi = cos(phi); Ctheta = cos(theta); Cpsi = cos(psi);
    
R = [Cpsi*Ctheta, Cpsi*Stheta*Sphi-Spsi*Cphi, Cpsi*Stheta*Cphi + Spsi*Sphi;
     Spsi*Ctheta, Spsi*Stheta*Sphi+Cpsi*Cphi, Spsi*Stheta*Cphi - Cpsi*Sphi;
     -Stheta, Ctheta*Sphi, Ctheta*Cphi];

% Translational control error in the body coordinates
eB = R' * eG;
if sum(abs(eB)) > 0
    %eB = eB./norm(eB);
end

e_k = [eB(1); eB(2)];
%e_k = eG(1:2);

e_km1 = x(3:4);
I_km1 = x(5:6);
D_km1 = x(7:8);

K = param.K;
Ti = param.Ti;
Td = param.Td;
N = param.N;
h = param.h;

P_k = K .* e_k;
%D_k = (Td / (Td + N * h)) .* D_km1 - ((K * Td * N)/(Td + N * h)) .* (e_k - e_km1);
%I_k = I_km1 + (K * h / Ti) .* e_k;

D_k = Td.*(e_k-e_km1)/h;
angles = P_k + D_k;

x(1) = angles(1);
x(2) = -angles(1);
x(3:4) = e_k;
%x(5:6) = I_k;
x(7:8) = D_k;
sys = x;

function sys = mdlOutputs(x)
sys = x(1:4);
