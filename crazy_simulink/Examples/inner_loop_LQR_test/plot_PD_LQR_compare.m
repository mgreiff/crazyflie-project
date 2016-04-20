figure(1);
subplot(211)
hold on;
title('Elevation, z, and yaw, \phi, of the open loop system using stabilising PD and stabilising LQR ')
plot(LQR_data.Time,LQR_data.Data(:,3),'b')
plot(PD_Data.Time,PD_Data.Data(:,3),'r')
plot(Ref_Data.Time,Ref_Data.Data(:,3),'k')
legend('LQR','PD','Reference','Location','NorthWest')
ylabel('z(t) [m]')
xlabel('Time, t [s]')

subplot(212)
hold on;
plot(LQR_data.Time,LQR_data.Data(:,7),'b')
plot(PD_Data.Time,PD_Data.Data(:,7),'r')
plot(Ref_Data.Time,Ref_Data.Data(:,7),'k')
legend('LQR','PD','Reference','Location','NorthWest')
ylabel('\phi(t), [rad]')
xlabel('Time, t [m]')