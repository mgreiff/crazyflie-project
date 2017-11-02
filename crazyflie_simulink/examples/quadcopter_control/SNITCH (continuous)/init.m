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


%% Run Simulation
sim('inner_pid_discrete_test')

% %%  For plotting
% load('reference')
% load('response')
% 
% %%% Position Plots
% figure(1)
% title('PID Control position response')
% subplot(3,1,1)
% plot(reference.Time,reference.Data(:,1),response.Time,response.Data(:,1),'LineWidth',3)
% xlabel('Time(s)')
% ylabel('X Position (m)')
% % legend('reference','response')
% subplot(3,1,2)
% plot(reference.Time,reference.Data(:,2),response.Time,response.Data(:,2),'LineWidth',3)
% xlabel('Time(s)')
% ylabel('Y Position (m)')
% % legend('reference','response')
% subplot(3,1,3)
% plot(reference.Time,reference.Data(:,3),response.Time,response.Data(:,3),'LineWidth',3)
% xlabel('Time(s)')
% ylabel('Z Position (m)')
% legend('reference','response')
% 
% figure(2)
% title('PID Control orientation response')
% subplot(3,1,1)
% plot(reference.Time,reference.Data(:,4),response.Time,response.Data(:,4),'LineWidth',3)
% xlabel('Time(s)')
% ylabel('\phi (rad)')
% legend('reference','response')
% subplot(3,1,2)
% plot(reference.Time,reference.Data(:,5),response.Time,response.Data(:,5),'LineWidth',3)
% xlabel('Time(s)')
% ylabel('\theta (rad)')
% % legend('reference','response')
% subplot(3,1,3)
% plot(reference.Time,reference.Data(:,6),response.Time,response.Data(:,6),'LineWidth',3)
% xlabel('Time(s)')
% ylabel('\psi (rad)')
% % legend('reference','response')

