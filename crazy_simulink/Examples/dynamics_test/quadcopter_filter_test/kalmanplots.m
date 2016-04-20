figure(1);
for ii = 1:3
    subplot(3,1,ii)
    hold on;
    plot(kalman_data.Time, kalman_data.Data(:,ii),'k--','LineWidth',1)
    plot(kalman_data.Time, kalman_data.Data(:,ii+3),'r')
    plot(ekf_data.Time, ekf_data.Data(:,ii+3),'b')
    legend('True','KF','EKF')
    if ii == 1
        title('Estimated and true angular speeds as a function of time')
        ylabel('\phi(t)')
    elseif ii == 2
        ylabel('\theta(t)')
    elseif ii == 3
        ylabel('\psi(t)')
    end
end

figure(2)
hold on;
plot(ekf_data.Time, abs(ekf_data.Data(:,ii+3) - kalman_data.Data(:,ii)),'b--')
plot(ekf_data.Time, abs(kalman_data.Data(:,ii+3) - kalman_data.Data(:,ii)),'r--')