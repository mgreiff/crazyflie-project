Ixx = I(1); Iyy=I(2); Izz = I(3);

Ac = zeros(12);
Ac(1:3,4:6) = eye(3);
Ac(4:6,4:6) = -diag(A)/m;
Ac(7:9,10:12) = eye(3);

Bc = zeros(12, 4);
Bc(6, :) = k;
Bc(10:12,:) = [0,-k*l/Ixx,0,k*l/Ixx;
              -k*l/Iyy,0,k*l/Iyy,0;
              -b/Izz,b/Izz,-b/Izz,b/Izz];
Bc = sqrt(m*g/k).*Bc;

Cc = eye(nStates);
Cc(10:12,:) = []; % angular speeds

Dc = zeros(length(Cc(:,1)), 4);


KF_h = 0.1; % discretization time
contsys = ss(Ac,Bc,Cc,Dc);
discsys = c2d(contsys, KF_h);

% Linearized model for use in the kalman filter
[KF_A, KF_B, KF_C, KF_D] = ssdata(discsys);

% Number of states
nStates = 12;

KF_P0 = eye(nStates); % Initial covariance matrix, tbd
KF_Q = eye(nStates);
KF_Q(12,12) = 0.1;

KF_R = diag([0.1,0.1,0.1,0.1,0.1,0.1,0.01,0.01,0.01]);

if length(KF_R) ~= length(KF_C(:,1))
    disp('The number of measured states in C does not match KF_R')
end

% Initial x
KF_x0 = zeros(12,1);