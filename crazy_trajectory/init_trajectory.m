close all;
clear all;

% Add simulink folders to path
addpath(genpath('.'));

load('trajectoryData')

% Required to run the example_simulink_trajectory_1.m example
Ts = 0.2;

% Required to run the example_simulink_trajectory_2.m example
splines.h = Ts;