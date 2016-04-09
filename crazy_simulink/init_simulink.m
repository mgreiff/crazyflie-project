close all;
clear all;

% Add simulink folders to path
addpath(genpath('.'));

% Initiate quadcopter model
run('quadcopter_init'); % /model

% Initiate PD baseline controller
run('init_pd_controller'); % /PD_controller

% Initiate PD posiiton controller
run('init_pd_position_controller'); % /PD_controller