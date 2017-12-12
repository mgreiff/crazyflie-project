% %% Drone 1 (LQR Control)
% run('init_inner_LQR')
% max_ang_ref = 0.5;
% 
% %% Drone 2 (Diff Flatness)
% % trajectory2 = load('trajTypeAZeroStart.mat');
% trajectory2 = load('trajTypeB.mat');
% trajectory2.h = inner_h;


%% For Abhijit's Model
load('2D_collisionavoidance_trajs_abhijit.mat')
drone1_traj = testwps{1};
drone2_traj = testwps{2};