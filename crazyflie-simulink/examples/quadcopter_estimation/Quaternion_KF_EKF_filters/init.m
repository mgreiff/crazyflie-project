%% Initializes controllers
run('init_quadcopter_model')
run('init_inner_controller');
run('init_inner_PD_c')

trajectory = load('trajTypeAZeroStart.mat');
trajectory.h = inner_h;

% Computes the saturation bounds for the thrust PD controller
sat_lim = 0.1;        % difference between thrust and omega saturation
omega_min_lim = 0;    % rad/s
omega_max_lim = 2500; % rad/s
Tm = 4.*k.^2.*omega_min_lim;
Tp = 4.*k.*omega_max_lim.^2;
T_min_lim = Tm + (Tp - Tm) * sat_lim;
T_max_lim = Tp - (Tp - Tm) * sat_lim;
tau_max_lim = 1e-5;
tau_min_lim = -1e-5;

%% Function handle for generating the linearized system in the EKF
D = 0.25;
KFparam.linSys = @(q, w, csig) getLinSys_Q(q(1),q(2),q(3),q(4),...
                                           w(1),w(2),w(3),...
                                           csig(1), m, I(1), I(2), I(3), A(1), A(2), A(3));

%% Function handle for the non-linear quadcopter dynamics used in the UKF and GPF
KFparam.dynSys = @(q, w) getDynMat_Q(q(1),q(2),q(3),q(4),...
                                     w(1),w(2),w(3),...
                                     m, g, I(1), I(2), I(3), A(1), A(2), A(3));

%% Parameters common for all filters
KF_C = eye(13);
KF_C = KF_C([3,4,5,7,8,9,10,11,12,13],:); % Only include dxi, deta
KF_variance = diag([0.00025, 0.049,0.049,0.05, 0.05, 0.05, 0.05, 0.005, 0.005,0.005]);
KF_mean = zeros(13,1);

%% Prameters for the Kalman Filter (KF)
KFparam.nDiscreteStates = 13;
KFparam.nControlsignals = 4;
KFparam.nMeasuredStates = size(KF_C, 1);
KFparam.nInputs = KFparam.nControlsignals + KFparam.nMeasuredStates;
KFparam.nOutputs = KFparam.nDiscreteStates;
KFparam.x0 = zeros(KFparam.nDiscreteStates, 1);
KFparam.x0(3) = 0.1;
KFparam.x0(7) = 1; % First index of the quaternion set to 1
KFparam.h = 0.02;

% Linearises the system around a stable hovering point
qlin = [1, 0, 0, 0];
wlin = [0, 0, 0];
csiglin = [m*g, 0, 0, 0];
[Atilde, Btilde] = KFparam.linSys(qlin, wlin, csiglin);
[KFparam.Ad, KFparam.Bd, KFparam.Cd, ~] = ssdata(c2d(ss(Atilde, Btilde, KF_C, 0), KFparam.h));
KFparam.Q = 1.*eye(KFparam.nDiscreteStates);
KFparam.R = 10.*eye(KFparam.nMeasuredStates);
KFparam.P0 = 1.*eye(KFparam.nDiscreteStates);
KFparam.P0(1:3,1:3) = 1000*eye(3);

%% Parameters for th3e Extended Kalman Filter (EKF)
EKFparam = KFparam;
EKFparam.Q = 0.3*eye(EKFparam.nDiscreteStates);
EKFparam.R = 0.1.*eye(EKFparam.nMeasuredStates);
EKFparam.R(3,3) = 0.1;
EKFparam.R(4,4) = 0.3;
EKFparam.R(5,5) = 0.3;
EKFparam.R = KF_variance;

%% Parameters for the unscented Kalman filter
UKFparam = EKFparam;
UKFparam.Cd  = KF_C;
UKFparam.Q = 0.*eye(KFparam.nDiscreteStates);
UKFparam.R = diag(KF_variance);%KF_variance;
UKFparam.P0 = 100.*eye(KFparam.nDiscreteStates);
UKFparam.beta = 2;     % Optimal for gaussian distributions
UKFparam.alpha = 0.1;  % Tuning parameter (0 < alpha < 1e-4)
UKFparam.keta = 0;     % Tuning parameter (set to 0 or 3-L)

%% Filter analysis parameters
measurePlot = [1,2];
measureMat = eye(size(KF_C, 1));
measureMat = measureMat(measurePlot,:);

filterPlot = [1,2,3];
filterMat = eye(KFparam.nDiscreteStates);
filterMat = filterMat(filterPlot,:);
