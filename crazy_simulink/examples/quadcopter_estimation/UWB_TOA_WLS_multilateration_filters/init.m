%% Initializes controllers
run('init_quadcopter_model')
run('init_inner_PD_d')
run('init_inner_controller')

trajectory = load('trajTypeAZeroStart.mat');
trajectory.h = inner_h;

% Initialize time lag references
TDOAparam.fRR = 1/60;       % Round robin frequency [s], here 60 [Hz]
TDOAparam.c = 3e8;          % Speed of light

% Anchor positions corresponding to the .yaml configuration file
aP = [0,0,2;
      3,0,0;
      6,0,2;
      6,3,0;
      3,3,2;
      0,3,0];
TDOAparam.anchorPos = aP;
TDOAparam.noiseMean = zeros(1, size(aP,1)); % Noise mean
TDOAparam.noiseStd = (1e-2.*max(max(aP))/TDOAparam.c); % Noise variance

if 1
    hold on;
    for ii =1:length(TDOAparam.anchorPos(:,1))
        plot3(TDOAparam.anchorPos(ii,1), TDOAparam.anchorPos(ii,2), TDOAparam.anchorPos(ii,3), '*r')
        text(TDOAparam.anchorPos(ii,1), TDOAparam.anchorPos(ii,2), TDOAparam.anchorPos(ii,3),['  # ',num2str(ii)])
    end
end