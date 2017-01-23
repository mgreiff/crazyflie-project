%% Initializes controllers
run('init_quadcopter_model')
run('init_inner_PD_d')
run('init_inner_controller')

%% Function handle for generating the linearized system in the EKF
D = 0.25;
KFparam.linSys = @(eta, csig) getLinSys(eta(1), eta(2), eta(2),...
                                        eta(4), eta(5), eta(6),...
                                        csig(1), csig(2), csig(3), csig(4),...
                                        m, I(1), I(2), I(3), D);

%% Function handle for the non-linear quadcopter dynamics used in the UKF and GPF
KFparam.dynSys = @(eta) getDynMat(eta(1), eta(2), eta(2),...
                                  eta(4), eta(5), eta(6),...
                                  m, g, I(1), I(2), I(3), D);

%% Parameters common for all filters
KF_C = eye(12);
KF_C = KF_C([4,5,6,10,11,12],:); % Only include dxi, deta
KF_variance = diag([0.25, 0.25, 0.25, 0.005, 0.005, 0.005]);
KF_mean = zeros(12,1);

%% Prameters for the Kalman Filter (KF)
KFparam.nDiscreteStates = 12;
KFparam.nControlsignals = 4;
KFparam.nMeasuredStates = size(KF_C, 1);
KFparam.nInputs = KFparam.nControlsignals + KFparam.nMeasuredStates;
KFparam.nOutputs = KFparam.nDiscreteStates;
KFparam.x0 = zeros(KFparam.nDiscreteStates, 1);
KFparam.h = 0.05;

% Linearises the system around a stable hovering point
etalin = [0, 0, 0, 0, 0, 0];
csiglin = [m*g, 0, 0, 0];
[Atilde, Btilde] = KFparam.linSys(etalin, csiglin);
[KFparam.Ad, KFparam.Bd, KFparam.Cd, ~] = ssdata(c2d(ss(Atilde, Btilde, KF_C, 0), KFparam.h));
KFparam.Q = 1.*eye(KFparam.nDiscreteStates);
KFparam.R = 1000.*eye(KFparam.nMeasuredStates);
KFparam.P0 = 100.*eye(KFparam.nDiscreteStates);

%% Parameters for th3e Extended Kalman Filter (EKF)
EKFparam = KFparam;
EKFparam.Q = 10.*eye(EKFparam.nDiscreteStates);
EKFparam.R = 1.*eye(EKFparam.nMeasuredStates);

%% Parameters for the unscented Kalman filter
UKFparam = EKFparam;
UKFparam.Cd  = KF_C;
UKFparam.Q = 0.*eye(KFparam.nDiscreteStates);
UKFparam.R = KF_variance;
UKFparam.P0 = 10.*eye(KFparam.nDiscreteStates);
UKFparam.beta = 2;     % Optimal for gaussian distributions
UKFparam.alpha = 0.1;  % Tuning parameter (0 < alpha < 1e-4)
UKFparam.keta = 0;     % Tuning parameter (set to 0 or 3-L)

%% Filter analysis parameters
measurePlot = [1,2,3]+3;
measureMat = eye(size(KF_C, 1));
measureMat = measureMat(measurePlot,:);

filterPlot = [1,2,3]+6;
filterMat = eye(KFparam.nDiscreteStates);
filterMat = filterMat(filterPlot,:);
