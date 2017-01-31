%% Set up system linearized continuous system around the stable hovering point
% state vector x = [z, \dot z,\phi, \theta, \psi,\dot\phi, \dot\theta, \dot\psi]
run('init_quadcopter_model')
Ixx = I(1);
Iyy = I(2);
Izz = I(3);
Dii = 1e-5;

[Amat, Bmat] = getLinSys(0,0,0,0,0,0,m*g,0,0,0,m,Ixx,Iyy,Izz,Dii);
%[Amat, Bmat] = getDynMat(0,0,0,0,0,0,m,g,Ixx,Iyy,Izz,Dii);

max_lim = 1000*[0.45,0.1,2500];
min_lim = 1000*[-0.05,-0.1,0];
factor = 1e-3;
Q = eye(size(Amat));
Q(1, 1) = 1e3;
Q(2, 2) = 1e3;
Q(3, 3) = 1e5;
Q(4, 4) = 1e3;
Q(5, 5) = 1e3;
Q(6, 6) = 1e4;
Q(7, 7) = 1e8;
Q(8, 8) = 1e8;
Q(9, 9) = 1e8;
Q(10, 10) = 1e7;
Q(11, 11) = 1e7;
Q(12, 12) = 1e7;

R = eye(size(Bmat,2));
R(1, 1) = 1e5;
R(2, 2) = 1e8;
R(3, 3) = 1e8;
R(4, 4) = 1e8;

Q=Q*factor;
R=R*factor;

[K,S,E] = lqr(Amat,Bmat,Q,R);