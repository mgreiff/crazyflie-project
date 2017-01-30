function [sys,x0,str,ts] = madgwick_bias_sfunc(t,x,u,flag,param)

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
sizes.NumDiscStates  = 7;
sizes.NumOutputs     = 4; % 4 dof - quaternion
sizes.NumInputs      = 6; % 6 dof - accelerometer (a_G) and gyro (w_B)
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes); 

x0 = [1,0,0,0,0,0,0]';  % Initial quaternion and biases
str = [];                % Set str to an empty matrix.
ts  = [param.h 0];       % sample time: [period, offset]
		      
%==============================================================
% Update the discrete states
%==============================================================
function sys = mdlUpdates(x,u,param)
h = param.h;
beta = param.beta;
zeta = param.zeta;

aG = u(1:3);
wB = u(4:6);
q = x(1:4);
w_bias = x(5:7);

% Normalize the accelerometer measurements
aG = aG./sqrt(aG'*aG);

% Compute the objective function and Jacobian
f = [-2 * (q(2) * q(4) - q(1) * q(3)) - aG(1);
     -2 * (q(1) * q(2) + q(3) * q(4)) - aG(2);
     -2 * (0.5 - q(2) * q(2) - q(3) * q(3)) - aG(3)];

J = -2 .* [-q(3), q(4), -q(1), q(2);
           q(2),  q(1),  q(4), q(3);
           0, -2 * q(2), -2 * q(3), 0];

% Compute the gradient (matrix multiplication)
SEqHatDot = J' * f;

% Normalise the gradient
SEqHatDot = SEqHatDot./sqrt(SEqHatDot'*SEqHatDot);

% compute angular estimated direction of the gyroscope error
w_err_x = 2 * q(1) * SEqHatDot(2) - 2 * q(2) * SEqHatDot(1) - 2 * q(3) * SEqHatDot(4) + 2 * q(4) * SEqHatDot(3);
w_err_y = 2 * q(1) * SEqHatDot(3) + 2 * q(2) * SEqHatDot(4) - 2 * q(3) * SEqHatDot(1) - 2 * q(4) * SEqHatDot(2);
w_err_z = 2 * q(1) * SEqHatDot(4) - 2 * q(2) * SEqHatDot(3) + 2 * q(3) * SEqHatDot(2) - 2 * q(4) * SEqHatDot(1);

w_err = [w_err_x; w_err_y; w_err_z];

% compute and remove the gyroscope baises
w_bias = w_bias + h .* zeta .* w_err;
wB = wB - w_bias;

% Compute the quaternion derrivative measured by gyroscopes
temp = 0.5.*[-q(2), -q(3), -q(4);
              q(1), -q(4),  q(3);
              q(4),  q(1), -q(2);
             -q(3),  q(2),  q(1)];
SEqDot_omega = temp*wB;

% Compute and integrate the estimated quaternion derivative
q = q + (SEqDot_omega - (beta .* SEqHatDot)) .* h;

% Normalise quaternion
q = q/sqrt(q'*q);

x(1:4) =  q;
x(5:7) = w_bias;
sys = x;

%==============================================================
% Calculate outputs
%==============================================================
function sys = mdlOutputs(x)
sys = x(1:4);
