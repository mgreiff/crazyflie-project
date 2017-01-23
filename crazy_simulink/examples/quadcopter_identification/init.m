load('k_mapping_data')

T = 9.81.*data(:,2)/1000;
RPM = data(:,5);
omega = RPM*2*pi/60;
PWM = data(:,4)*0.01;
varcoeff = @(a,x)(a(1).*x.^2 +a(2).*x + a(3));
zerocoeff = @(a,x)(a.*x.^2);

zerobeta = nlinfit(omega,T,zerocoeff,[1e-07]);
disp(sum((zerobeta.*omega.^2 - T).^2)/length(T))
varbeta = nlinfit(omega,T,varcoeff,[1e-07, -1e-07, 1e-07]);
disp(sum((varbeta(1).*omega.^2 + varbeta(2).*omega + varbeta(3)- T).^2)/length(T))

hold on;
tt = linspace(0, max(omega)+200,1000);
plot(omega, T, 'xk')
plot(tt, zerobeta.*tt.^2, 'b')
plot(tt, varbeta(1).*tt.^2 + varbeta(2).*tt + varbeta(3), 'r')

figure(2);
pwmcoeff = @(a,x)(a(1).*x.^2 +a(2).*x.^1);
tt = linspace(0, 1,1000);
beta = nlinfit(PWM,T/4,pwmcoeff,[1e-10, 1e-07]);
hold on;
plot(PWM, T, 'xk')
plot(tt, beta(1).*tt.^2 + beta(2).*tt, 'r')