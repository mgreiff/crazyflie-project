%% Set up basic model
Ad = diag(A);
a1 = 4.8; a2 = 8.32; b = 8.32;
a1 = KetaD(1); a2 = KetaP(1); b = KetaP(1);

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

disp('Generate CVXGEN data struct...')
% Must be specified to run the solver eg using
% 
%   [vars, status] = csolve(CVXparameters)
%

% Dynamics and initial conditions
CVXparameters.A = Ad;
CVXparameters.B = Bd;
CVXparameters.x_0 = [1;1;1;0;0;0;0;0;0;0];

% Design parameters
CVXparameters.Q = eye(10);
CVXparameters.R = eye(3);
CVXparameters.S_max = 1000;
CVXparameters.u_max = 10000;
CVXparameters.predictionHorizon = 10;

% Must be specified to make the S-fucntion run
CVXparameters.nDiscreteStates = 10;
CVXparameters.nOutputs = 3;
CVXparameters.nInputs = CVXparameters.nDiscreteStates*(CVXparameters.predictionHorizon+2);
CVXparameters.h = Ts; % Stepsize
disp('...complete!')

%% Should be updated within simulink, just included here to make the file run
%  where positions change(on the prediction horizon) but the
%  speeds and angular parameters are kept constant. 

CVXparameters.r_0 = [1.0,1.00,1.0,0,0,0,0,0,0,0]';
CVXparameters.r_1 = [1.1,0.90,1.0,0,0,0,0,0,0,0]';
CVXparameters.r_2 = [1.2,0.81,1.1,0,0,0,0,0,0,0]';
CVXparameters.r_3 = [1.3,0.73,1.1,0,0,0,0,0,0,0]';
CVXparameters.r_4 = [1.4,0.65,1.2,0,0,0,0,0,0,0]';
CVXparameters.r_5 = [1.5,0.55,1.2,0,0,0,0,0,0,0]';
CVXparameters.r_6 = [1.6,0.49,1.2,0,0,0,0,0,0,0]';
CVXparameters.r_7 = [1.7,0.42,1.1,0,0,0,0,0,0,0]';
CVXparameters.r_8 = [1.8,0.39,1.1,0,0,0,0,0,0,0]';
CVXparameters.r_9 = [1.9,0.38,1.0,0,0,0,0,0,0,0]';
CVXparameters.r_10 = [2.0,0.38,1.0,0,0,0,0,0,0,0]';
