% Generate function handles for the linearized quad rotor system without
% PID control

syms x y z dx dy dz ddx ddy ddz qw qx qy qz dqw dqx dqy dqz wx wy wz dw1 dw2 dw3 T tauphi tautheta taupsi m g Ixx Iyy Izz Axx Ayy Azz 'real'

% Gravity acceleration
Ga = -g * [0; 0; 1];

xi = [x;y;z];
xidot = [dx;dy;dz];
xiddot = [ddx;ddy;ddz];
q = [qw; qx; qy; qz];
qdot = [dqw; dqx; dqy; dqz];
w = [wx; wy; wz];
wdot = [dw1; dw2; dw3];
qv = [qx; qy; qz];
tau = [tauphi; tautheta; taupsi];

omegaw = 0; omegav = w;
    
angM = [2*(qx*qz + qw*qy);
        2*(qy*qz - qw*qx);
        qw.^2 - qx.^2 - qy.^2 + qz.^2];

Ta = T/m * angM;

% Drag
A = [Axx, 0, 0; 0, Ayy, 0; 0, 0, Azz];
Aa = -1/m .* A * xidot;
I = [Ixx, 0, 0; 0, Iyy, 0; 0, 0, Izz];

qvx = [0, -qz, qy;  qz, 0, -qx; -qy, qx, 0];
QL1 = -qv';
QL2 = eye(3)*qw + qvx;
Iwx = [0, -Izz*wz, Iyy*wy;  Izz*wz, 0, -Ixx*wx; -Iyy*wy, Ixx*wx, 0];

A1 = zeros(13,3);
A2 = [eye(3);-(1/m).*A.*eye(3);zeros(7,3)];
A3 = zeros(13,4);
A4 =  [zeros(6,3); 0.5.*QL1; 0.5*QL2; inv(I)*Iwx];
Ac = [A1, A2, A3, A4];

B1 = [zeros(3,1); 1/m .* angM; zeros(7,1)];
B2 = [zeros(10,3); inv(I)];
Bc = [B1, B2];

Gc = [zeros(5,1); -g; zeros(7,1)];

z = [xi;xidot;q;w];

u = [T;tau];
h = Ac*z + Bc*[T; tau];
f = [xidot;
     -1/m * A* xidot + T/m * angM - [0;0;g];
     0.5.*[omegaw'*qw - qv'*omegav; omegav.*qw + omegaw.*qv + cross(omegav, qv)];
     I\(tau - cross(omegav, I*omegav))];

Amat = jacobian(f,z);
Bmat = jacobian(f,u);

%Write the non-linear dynamics to a file
%matlabFunction(f ,'file','getDynSys.m','vars',{'x', 'y', 'z', 'dx', 'dy', 'dz', 'phi', 'theta', 'psi', 'dphi', 'dtheta', 'dpsi', 'T', 'tauphi', 'tautheta', 'taupsi', 'm', 'g', 'Ixx', 'Iyy', 'Izz', 'A'},'outputs',{'dstates'});

%Write the non-linear dynamical matrices to a file
matlabFunction(Ac, Bc, Gc,'file','getDynMat_Q.m','vars',{'qw', 'qx', 'qy', 'qz', 'wx', 'wy', 'wz', 'm','g','Ixx', 'Iyy', 'Izz', 'Axx', 'Ayy', 'Azz'},'outputs',{'Ac','Bc','Gc'});

%Write the linearized dynamics to a file
matlabFunction(Amat,Bmat,'file','getLinSys_Q.m','vars',{'qw', 'qx', 'qy', 'qz', 'wx', 'wy', 'wz', 'T', 'm', 'Ixx', 'Iyy', 'Izz', 'Axx', 'Ayy', 'Azz'},'outputs',{'Amat','Bmat'});


