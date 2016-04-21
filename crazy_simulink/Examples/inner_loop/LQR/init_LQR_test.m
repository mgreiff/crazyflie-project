%% Set up system linearized continuous system around the stable hovering point
% state vector x = [z, \phi, \theta, \psi, \dot z,\dot\phi, \dot\theta, \dot\psi]
Ixx = I(1);
Iyy = I(2);
Izz = I(3);

Am = [0,0,0,0,1,0,0,0;
      zeros(3,5),eye(3);
      0,0,0,0,-A(3)/m,0,0,0;
      zeros(3,8)];

Bm = sqrt(g*m/k).*[zeros(4,4);
                   k,        k,        k,       k;
                   0,        -k*l/Ixx, 0,       k*l/Ixx;
                   -k*l/Iyy, 0,        k*l/Iyy, 0;
                   -b/Ixx,   b/Ixx,    -b/Ixx,  b/Ixx];
Cm = eye(8);

sys = ss(Am,Bm,Cm,[]);

% Puts large weights on the 0th derivatives
Q = eye(size(Am));

Q(1,1) = 1000;
Q(2,2) = 1;
Q(3,3) = 1;
Q(4,4) = 100;
Q(5,5) = 100;
Q(6,6) = 1;
Q(7,7) = 1;
Q(8,8) = 100;

R = 0.01*eye(size(Bm,2));
[K,S,E] = lqr(sys,Q,R);

LQR_K = 0.17;
LQR_I = 0;
LQR_D = 0.15;