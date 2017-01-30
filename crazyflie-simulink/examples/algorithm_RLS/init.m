% RLS - Demonstrates the on-line identification of parameters
% in a 3x3 pulse transfer function corresonding to the voltage
% to current function in the rotor dynamics. Demonstrates both
% the constant trace and reglar RLS formulations.

run('init_rotor_model')
run('init_rotor_PID')
RLSparam.lambda = 0.96;
RLSparam.h = 0.002;
RLSparam.x0 = zeros(5,1);
RLSparam.delta = 0.0001;
RLSparam.nA = 3;
RLSparam.nB = 2;

% Only applicable in the regularized trace algorithm
trRLSparam.lambda = 0.95;
trRLSparam.h = 0.002;
trRLSparam.x0 = zeros(5,1);
trRLSparam.delta = 0.1;
trRLSparam.c1 = 10000;
trRLSparam.c2 = trRLSparam.c1/1e7;
trRLSparam.nA = 3;
trRLSparam.nB = 2;