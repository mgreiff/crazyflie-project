%% Set up basic model
Ad = diag(A);
a1 = 4.8; a2 = 8.32; b = 8.32; % Bad estimates
a1 = KphiD; a2 = KphiP; b = KphiP;


T0 = m*g;

Am1 = [zeros(10,3)];
Am2 = [eye(3); -Ad/m; zeros(4,3)];
Am3 = [zeros(3,2); T0/m 0; 0 -T0/m; zeros(3,2); -a2*eye(2)];
Am4 = [zeros(6,2); eye(2); -a1*eye(2)];
Am = [Am1 Am2 Am3 Am4];

Bm = zeros(10, 3);
Bm(9, 1) = b;
Bm(10, 2) = b;
Bm(6, 3) = 1/m;

Cm = zeros(5, 10);
Cm(1:3, 1:3) = eye(3);
Cm(4:5, 7:8) = eye(2);

Dm = zeros(5, 3);

%% Discretize
Ts = 0.2; % sample time

[Ad, Bd, Cyd, Dzd]=ssdata(c2d(ss(Am,Bm,Cm,Dm),Ts));
Czd = Cyd;
Ccd = Cyd;
Dcd = Dzd;
Cyd = [Cyd; zeros(3, 10)]
Cyd(6:8, 4:6) = eye(3);

% % % Controller Design % % %

%% Constraints
% limit delta u
du_max = [inf inf inf]';
du_min = [-inf -inf -inf]';
% limit absolute value of u
angle_max = 35 * pi/180;
u_max = [angle_max angle_max inf]';
u_min = [-angle_max -angle_max -inf]';
% limit constrained outputs
z_max = 2*[inf inf inf angle_max angle_max]';
z_min = 2*[-inf -inf -inf -angle_max -angle_max]';

% Prediction parameters
Hp = 30;  % Prediction horizon
Hu = 30;  % Horizon for varying input signal
Hw = 0;  % First penalty sample
zblk= 1;  % blocking factor 
ublk= 1;  % blocking factor


% Weights
Q = diag([2 2 2 0.1 0.1]);
R = 20*diag([1 1 1]);
W =  0.5*diag([1 1 1 1 1 1 1 1 1 1]);
V =  0.01*diag([1 1 1 1 1 1 1 1]);


% Create controller
md_quadcopter = MPCInit(Ad,Bd,Cyd,Czd,Dzd,Ccd,Dcd,Hp,Hw,zblk,Hu,ublk, ...
	    du_max,du_min,u_max,u_min,z_max, ...
	    z_min,Q,R,W,V,Ts,2,'qp_as');
