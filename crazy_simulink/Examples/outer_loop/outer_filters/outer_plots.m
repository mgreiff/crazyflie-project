load('outer_AKF.mat');
load('outer_KF.mat');
load('outer_TN.mat');

figure(1)
subplot(2,2,1)
hold on;
plot(TN.Time,TN.data(:,3),'b');
plot(TN.Time,TN.data(:,4),'r');
legend('Position, x(t)','Acceleration, a(t)')
title('Measurements in 1D-acceleration and 1D-position\n as seen on the host computer')
xlabel('Time [s]')
ylabel('x(t),a(t)')

subplot(2,2,2)
hold on;
plot(KF.Time,KF.data(:,4),'b');
plot(AKF.Time,AKF.data(:,4),'r');
plot(TN.Time,TN.data(:,1),'k','LineWidth',1);
legend('KF','AKF','True')
title('Estimates of 1D-position as seen on the crazyflie')
xlabel('Time [s]')
ylabel('x_{hat}(t)')

subplot(2,2,3)
hold on;
plot(KF.Time,KF.data(:,1),'b','LineWidth',1);
plot(AKF.Time,AKF.data(:,1),'r','LineWidth',1);
plot(TN.Time,TN.data(:,1),'k','LineWidth',1);
legend('KF','AKF','True')
title('MA-filtered estimate of 1D-position as seen on the crazyflie')
xlabel('Time [s]')
ylabel('x_{hat}(t)')

subplot(2,2,4)
hold on;
plot(KF.Time,sqrt((TN.data(:,1) - KF.data(:,1)).^2),'b');
plot(AKF.Time,sqrt((TN.data(:,1) - AKF.data(:,1)).^2),'r');
legend('KF','AKF')
title('RMS error in the 1D-position estimate on the crazyflie')
xlabel('Time [s]')
ylabel('RMS(x(t) - x_{hat}(t))')