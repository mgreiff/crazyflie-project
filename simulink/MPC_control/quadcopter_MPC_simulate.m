%% Simulate linearized system
if 1 % Visualize closed loop attitude dynamics step response (theta)
    figure(1)
    x0 = zeros(2,1);
    simtime = 1.4;
    t = linspace(0,simtime,simtime/Ts + 2);
    u = ones(1,length(t));
    [Yc,Tc,~] = lsim(syscont,u,t,x0);
    [Yd,Td,~] = lsim(sysdisc,u,t,x0);
    hold on;
    plot(Tc,Yc,'r')
    stem(Td,Yd,'b')
end

if 1 % Visualize closed loop attitude dynamics step responses
    figure(2)
    x0 = zeros(10,1);
    simtime = 1.4;
    N = simtime/Ts + 2;
    t = linspace(0,simtime,N);
    u = ones(length(t),3);
    u(:,2) = 0.5;
    u(1:ceil(N/2),3) = 0.5;
    u(ceil(N/2)+1:end,3) = -0.5;
    [Y,T,~] = lsim(syscomplete,u,t,x0);
    stairs(T,Y)
    legend('x','theta_1','y','theta_2','z')
end

%% TODO Simulate MPC control