%% Drone 1 (LQR Control)
run('init_inner_LQR')
max_ang_ref = 0.5;

%% Drone 2 (Diff Flatness)
% trajectory2 = load('trajTypeAZeroStart.mat');
trajectory2 = load('trajTypeB.mat');
trajectory2.h = inner_h;


