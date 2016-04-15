load('trajectoryData')
currentTime = 8;  % Starting time
Ts = 0.5;         % Time between samples
predHorizon = (20-8)/0.5; % Number of points on the prediction horizon

times = currentTime + Ts.* (0:predHorizon-1);
referenceTrajectory = zeros(6,predHorizon);

for ii = 1:predHorizon
    % Finds the spline to evaluate
    T = times(ii);
    splineNumber = sum(splines.times <=  T);

    % Treats the case when the time is outside the defined interval [t_0,t_s]
    if splineNumber == 0
        disp('Tried to evaluate spline before t_0, setting reference to the initial spline point.')
        T = splines.times(1);
        splineNumber = 1;
    elseif T >= splines.times(end)
        disp('Tried to evaluate spline after t_s, keeping position in terinal spline point')
        T = splines.times(end);
        splineNumber = length(coeff)/(N + 1);
    end
    % Evaluates splines and stores data in referenceTrajectory
    for dim = 1:3
        startIndex = (splines.N + 1) * (splineNumber - 1) + 1;
        endIndex = (splines.N + 1) * splineNumber;
        poscoeff = splines.coeff(startIndex:endIndex,dim);
        poscoeff = poscoeff(end:-1:1); % Reverses for polyval
        tt = T - splines.times(splineNumber);

        % Position
        referenceTrajectory(dim,ii) = polyval(poscoeff,tt);

        % Velocity
        poscoeff(end) = [];
        velcoeff = (length(poscoeff):-1:1)'.*poscoeff; 
        referenceTrajectory(dim + 1,ii) = polyval(velcoeff,tt);
    end
end

%% Plot the reference points
figure(1)
for ii = 1:3
    subplot(3,1,ii)
    plot(times, referenceTrajectory(ii,:), 'g*')
    plot_splines(splines.coeff(:,ii), splines.times , 'position')
end

figure(2)
plot3(referenceTrajectory(1,:), referenceTrajectory(2,:), referenceTrajectory(3,:), 'b')
