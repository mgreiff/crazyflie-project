run('init_inner_PD_d')
PD_position_param.K = 0.1;
PD_position_param.Td = 0.1;
PD_position_param.Ti = 100;
PD_position_param.N = 100;
PD_position_param.h = 0.002;

PosP = 0.2500;
PosI = 0;
PosD = 0.2;

% No I part
PosP = 0.20;
PosI = 0;
PosD = 0.24;

max_ang_ref = 0.5;