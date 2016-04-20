figure(1);
N = 1700;
for ii = 1:3
    subplot(3,1,ii)
    hold on;
    plot(KF_data.Time(1:N), KF_data.Data(1:N,ii),'k--','LineWidth',2)
    plot(KF_data.Time(1:N), KF_data.Data(1:N,ii+3),'r','LineWidth',1)
    plot(EKF_data.Time(1:N), EKF_data.Data(1:N,ii+3),'b','LineWidth',1)
    plot(UKF_data.Time(1:N), UKF_data.Data(1:N,ii+3),'g','LineWidth',1)

    if ii == 1
        title('Estimated and true angular speeds as a function of time')
        ylabel('\phi(t) [rad/s]')
        legend('True','KF','EKF','UKF')
        xlabel('Time [s]')
    elseif ii == 2
        ylabel('\theta(t) [rad/s]')
        xlabel('Time [s]')
    elseif ii == 3
        ylabel('\psi(t) [rad/s]')
        xlabel('Time [s]')
    end
end

figure(2);
for k = 1:3
    subplot(3,1,k)
    hold on;
    plot(KF_data.Time(1:N),  (KF_data.Data(1:N,k+3) - KF_data.Data(1:N,k)),'b','LineWidth',1)
    plot(EKF_data.Time(1:N), (EKF_data.Data(1:N,k+3) - EKF_data.Data(1:N,k)),'r','LineWidth',1)
    plot(UKF_data.Time(1:N), (UKF_data.Data(1:N,k+3) - UKF_data.Data(1:N,k)),'g','LineWidth',1)

    if k == 1
        title('Estimation error as a function of time')
        legend('True','KF','EKF','UKF')
        ylabel('\phi_{obs}(t) - \phi(t) [rad/s]')
        xlabel('Time [s]')
    elseif k == 2
        ylabel('\theta_{obs}(t) - \theta(t) [rad/s]')
        xlabel('Time [s]')
    elseif k == 3
        ylabel('\psi_{obs}(t) - \psi(t) [rad/s]')
        xlabel('Time [s]')
    end
end

Ts = 0.01
KF_error = sum(sum(abs((KF_data.Data(1:N,4:6) - KF_data.Data(1:N,1:3))))*Ts)
EKF_error = sum(sum(abs((EKF_data.Data(1:N,4:6) - EKF_data.Data(1:N,1:3))))*Ts)
UKF_error = sum(sum(abs((UKF_data.Data(1:N,4:6) - UKF_data.Data(1:N,1:3))))*Ts)

figure(3);
[AX,H1,H2] = plotyy(measurements.Time(1:N),measurements.Data(1:N,3),measurements.Time(1:N),measurements.Data(1:N,9));
title('Corrupted z- and \psi-measurements used in the state estimation')
set(AX,{'ycolor'},{'r';'b'})
set(H1,'color','red')
set(H2,'color','blue')
set(get(AX(1),'Ylabel'),'String','Corrupted z-measurement [m]') 
set(get(AX(2),'Ylabel'),'String','Corrupted \psi-measurement [rad/s]') 
xlabel('Time [s]')