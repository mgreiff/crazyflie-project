%% Set up system linearized continuous system around the stable hovering point
% state vector x = [z, \dot z,\phi, \theta, \psi,\dot\phi, \dot\theta, \dot\psi]
run('init_quadcopter_model')
Ixx = I(1);
Iyy = I(2);
Izz = I(3);
Dii = 1e-5;

[Amat, Bmat] = getLinSys(0,0,0,0,0,0,m*g,0,0,0,m,Ixx,Iyy,Izz,Dii);
%[Amat, Bmat] = getDynMat(0,0,0,0,0,0,m,g,Ixx,Iyy,Izz,Dii);

Amat([1,2,4,5],:)=[];
Amat(:,[1,2,4,5])=[];
Bmat([1,2,4,5],:)=[];

Q = eye(size(Amat));
Q(1, 1) = 1e4;
Q(2, 2) = 1e3;
Q(3, 3) = 1e2;
Q(4, 4) = 1e2;
Q(5, 5) = 1e2;
Q(6, 6) = 1e1;
Q(7, 7) = 1e1;
Q(8, 8) = 1e1;

R = eye(size(Bmat,2));
R(1, 1) = 1e3;
R(2, 2) = 1e7;
R(3, 3) = 1e7;
R(4, 4) = 1e7;

[K,S,E] = lqr(Amat,Bmat,Q,R);
