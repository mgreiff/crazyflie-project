run('init_KF_example')

nStates = 12;
nControlsignals = 4;
nMeasuredStates = 9;
EKF_param.nInputs = nControlsignals + nMeasuredStates;
EKF_param.nOutputs = nStates;
EKF_param.nDiscreteStates = nStates; % Add accelerations
EKF_param.x0 = zeros(EKF_param.nDiscreteStates,1);
EKF_param.h = 0.1;
EKF_param.g = g;
EKF_param.m = m;
EKF_param.k = k;
EKF_param.A = A;
EKF_param.I = I;
EKF_param.l = l;
EKF_param.b = b;