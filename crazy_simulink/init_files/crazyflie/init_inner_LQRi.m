%% Set up system linearized continuous system around the stable hovering point
% state vector x = [z, \phi, \theta, \psi, \dot z,\dot\phi, \dot\theta, \dot\psi]
Ixx = I(1);
Iyy = I(2);
Izz = I(3);

[Amat, Bmat] = getLinSys(0,0,0,0,0,0,m*g,0,0,0,m,Ixx,Iyy,Izz,Dii);

Amat([1,2,4,5],:)=[];
Amat(:,[1,2,4,5])=[];
Bmat([1,2,4,5],:)=[];

Cm = zeros(4,8);
Cm(1,1) = 1;
Cm(2:4,3:5) = eye(3);

% Augments the state space with integrator states on the elevation and
% position \dot x_i = [z, \phi, \theta, \psi] \in\mathbb{R}^{4x1} so that
% x_e = [x ; x_i]

Ai = [Amat, zeros(8,4);
      Cm, zeros(4)];
Bi = [Bmat; zeros(4,4)];
Ci = [Cm, zeros(4,4)];

% Puts large weights on the 0th derivatives
Q = 100*eye(size(Ai));
Q(1, 1) = 1e4;
Q(2, 2) = 1e3;
Q(3, 3) = 1e2;
Q(4, 4) = 1e2;
Q(5, 5) = 1e2;
Q(6, 6) = 1e1;
Q(7, 7) = 1e1;
Q(8, 8) = 1e1;
Q(9, 9) = 1e5;
Q(10, 10) = 1e3;
Q(11, 11) = 1e3;
Q(12, 12) = 1e3;

sys_i = ss(Ai,Bi,Ci,[]);
R = eye(size(Bi,2));
R(1, 1) = 1e3;
R(2, 2) = 1e7;
R(3, 3) = 1e7;
R(4, 4) = 1e7;

[K,S,E] = lqr(sys_i,Q,R);