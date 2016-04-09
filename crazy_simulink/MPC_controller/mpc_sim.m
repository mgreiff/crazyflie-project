n = 100;
t = 0:Ts:(n);
r = zeros(n, 5);
r(:, 2) = 2;
%r(25:end, 2) = 5;
d = zeros(n, 3);

[x, u, y, z, zPredTraj, uPredTraj] = MPCSim(md_quadcopter, r, d);
tz = t(1:size(z,1));

figure();
plot(tz, [z(:,2) z(:,5)]);
hold on;
zd = simmeasured.data;
plot(simmeasured.time, [zd(:,2) zd(:,5)]);
legend('y_mpc', 'phi_mpc', 'y', 'phi');
figure();
tu = t(1:size(u,1));
plot(tu, u(:, 2));
hold on;
plot(simu.time, simu.data(:, 1));
legend('u_mpc', 'u');