Po = 0.2; % P-part of position controller
Io = 0.0; % I-part of position controller
Do = 0.17; % D-part of position controller
N = 100; % Filter coeficcient for D-part

omegalim = [0,900]; % Limits the rotational speed
euleranglelim = [-35*pi/180, 35*pi/180]; % Euler angle limit

seed = 123; % Seed for random target generators