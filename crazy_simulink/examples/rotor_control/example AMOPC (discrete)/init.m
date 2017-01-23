% Only applicable in the regularized trace algorithm
trRLSparam.lambda = 0.99;
trRLSparam.h = 0.5;
trRLSparam.x0 = zeros(5,1);
trRLSparam.x0(1) = -1.5;
trRLSparam.x0(2) = 0.5;
trRLSparam.x0(3) = 0.2;
trRLSparam.x0(4) = 0.2;
trRLSparam.delta = 0.1;
trRLSparam.nA = 3;
trRLSparam.nB = 2;
trRLSparam.c1 = 10000;
trRLSparam.c2 = trRLSparam.c1/1e7;

%% MOPPC parameters
AMOPPCparam.h = 0.002;
AMOPPCparam.degR = 1;
AMOPPCparam.degT = 1;
AMOPPCparam.degS = 1;

% Observer pole
AMOPPCparam.ao = -0.99;

% Reference model
AMOPPCparam.bm0 = 0.1;

% Test with poles on point or slightly off
% slightly faster
AMOPPCparam.am1 = -1.943454396449151;
AMOPPCparam.am2 = 0.945009782207162;

% on point
%AMOPPCparam.am1 = -1.947644233748935;
%AMOPPCparam.am2 = 0.948696191846759;