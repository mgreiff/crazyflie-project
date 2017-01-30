
subplot(2,2,1);
hold on;
grid on;
plot(omega.Time', omega.Data(:,1), 'b', 'linewidth', 3)
plot(omega.Time', omega.Data(:,2), 'k', 'linewidth', 3)
plot(omega.Time', omega.Data(:,3), 'g', 'linewidth', 1)
plot(omega.Time', omega.Data(:,4), 'r', 'linewidth', 1)
legend('\Omega_1', '\Omega_2', '\Omega_3', '\Omega_4', 'location', 'NE')
xlabel('Time [s]')
ylabel('Rotor speed [rad/s]')
title('Rotor speeds')
axis([0,2.5,1650,1900])

subplot(2,2,2);
hold on;
grid on;
plot(Ttau.Time', Ttau.Data(:,1), 'm')
plot(Ttau.Time', 1000*Ttau.Data(:,2), 'r')
plot(Ttau.Time', 1000*Ttau.Data(:,3), 'b')
plot(Ttau.Time', 1000*Ttau.Data(:,4), 'g')
legend('T_z', '\tau_x', '\tau_y', '\tau_z')
axis([0,2.5,-0.25,0.75])
xlabel('Time [s]')
ylabel('Thrust [N] torques [Nm\cdot 10^{-3}]')
title('Resulting thrusts and torques')

subplot(2,2,3);
hold on;
grid on;
plot(states.Time', states.Data(:,1), 'r')
plot(states.Time', states.Data(:,2), 'b')
plot(states.Time', states.Data(:,3), 'g')
legend('x_k', 'dy_k', 'dz_k', 'location', 'NE')
plot(states.Time', states.Data(:,4), 'k')
plot(states.Time', states.Data(:,5), 'k')
plot(states.Time', states.Data(:,6), 'k')
plot(states.Time', states.Data(:,1), 'r')
plot(states.Time', states.Data(:,2), 'b')
plot(states.Time', states.Data(:,3), 'g')
axis([0,2.5,-3,3])
xlabel('Time [s]')
ylabel('Translational velocity [m/s]')
title('Speed in global frame by continuous model (black) and discrete time model')

subplot(2,2,4);
hold on;
grid on;
plot(states.Time', states.Data(:,1+6), 'r')
plot(states.Time', states.Data(:,2+6), 'b')
plot(states.Time', states.Data(:,3+6), 'g')
legend('\phi_k', '\theta_k', '\psi_k', 'location', 'NW')
plot(states.Time', states.Data(:,4+6), 'k')
plot(states.Time', states.Data(:,5+6), 'k')
plot(states.Time', states.Data(:,6+6), 'k')
plot(states.Time', states.Data(:,1+6), 'r')
plot(states.Time', states.Data(:,2+6), 'b')
plot(states.Time', states.Data(:,3+6), 'g')
xlabel('Time [s]')
ylabel('Euler angles [rad]')
title('Euler angles in global frame by continuous model (black) and discrete time model')

