addpath(genpath('.'));
% Load most recent data
load('inner_PD_ref.mat')
load('inner_PD_data.mat')
load('inner_LQR_ref.mat')
load('inner_LQR_data.mat')
load('inner_LQRi_ref.mat')
load('inner_LQRi_data.mat')

sIndices = [3;7;8;9]; % Positional states
labelIndices = [1,3]; % The states to plot
labels = {'Elevation, z(t) [m]','Pitch, \phi(t) [rad]','Roll, \theta(t) [rad]','Yaw, \psi(t) [rad]'};
figure(1);
for ii = 1:length(labelIndices)
    k = sIndices(ii);
    subplot(2,1,ii)
    hold on;
    title([labels{labelIndices(ii)} ', when stabilising the open loop system using PD, and LQR LQRi'])
    plot(PD_ref.Time,PD_ref.Data(:,k),'g','LineWidth', 1)
    plot(PD_data.Time,PD_data.Data(:,k),'k')
    plot(LQR_data.Time,LQR_data.Data(:,k),'r')
    plot(LQRi_data.Time,LQRi_data.Data(:,k),'b')
    legend('Reference','PD','LQR','LQRi','Location','NorthWest')
    ylabel(labels{labelIndices(ii)})
    xlabel('Time, t [s]')
end

labels = {'Elevation error, z_{ref}(t)-z(t)','Pitch error, \phi_{ref}(t)-\phi(t)','Roll error, \theta_{ref}(t)-\theta(t)','Yaw error, \psi_{ref}(t)-\psi(t)'};
figure(2);
for ii = 1:length(labelIndices)
    k = sIndices(ii);
    subplot(2,1,ii)
    hold on;
    title([labels{labelIndices(ii)}, ', when using PD, LQR and LQRi'])
    plot(PD_data.Time,PD_ref.Data(:,k) - PD_data.Data(:,k),'k')
    plot(LQR_data.Time,LQR_ref.Data(:,k) - LQR_data.Data(:,k),'r')
    plot(LQRi_data.Time,LQRi_ref.Data(:,k) - LQRi_data.Data(:,k),'b')
    legend('PD','LQR','LQRi','Location','NorthWest')
    ylabel(labels{labelIndices(ii)})
    xlabel('Time, t [s]')
    
    disp(['Stationary error in ', labels{labelIndices(ii)}])
    disp('PD')
    disp((PD_ref.Data(995:1000:5000,k) - PD_data.Data(995:1000:5000,k))')
    disp('LQR')
    disp((LQR_ref.Data(995:1000:5000,k) - LQR_data.Data(995:1000:5000,k))')
    disp('LQRi')
    disp((LQRi_ref.Data(995:1000:5000,k) - LQRi_data.Data(995:1000:5000,k))')
end