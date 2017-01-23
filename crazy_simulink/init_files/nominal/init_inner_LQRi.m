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

Cm = [eye(4), zeros(4)];

% Augments the state space with integrator states on the elevation and
% position \dot x_i = [z, \phi, \theta, \psi] \in\mathbb{R}^{4x1} so that
% x_e = [x ; x_i]

Ai = [Am, zeros(8,4);
      Cm, -eye(4)];
Bi = [Bm; zeros(4,4)];
Ci = [Cm, zeros(4,4)];

% Puts large weights on the 0th derivatives
Q = 100*eye(size(Ai));
Q(1,1) = 300000;
Q(2,2) = 1000;
Q(3,3) = 1000;
Q(4,4) = 10000;
Q(5,5) = 10000;
Q(6,6) = 100;
Q(7,7) = 100;
Q(8,8) = 10000;
Q(9,9) = 1000000;
Q(10,10) = 10000;
Q(11,11) = 10000;
Q(12,12) = 10000;

sys_i = ss(Ai,Bi,Ci,[]);

R = 1*eye(size(Bm,2));
[K,S,E] = lqr(sys_i,Q,R);

LQR_K = 0.17;
LQR_I = 0;
LQR_D = 0.15;