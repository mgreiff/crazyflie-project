% Generate function handles for the linearized quad rotor system without
% PID control

syms x y z dx dy dz ddx ddy ddz phi theta psi dphi dtheta dpsi ddphi ddtheta ddpsi T tauphi tautheta taupsi m g Ixx Iyy Izz A 'real'

% Gravity acceleration
Ga = -g * [0; 0; 1];

r = [x;y;z];
rdot = [dx;dy;dz];
rddot = [ddx;ddy;ddz];
eta = [phi;theta;psi];
etadot = [dphi;dtheta;dpsi];
etaddot = [ddphi;ddtheta;ddpsi];
taub = [tauphi; tautheta; taupsi];
% Thrust acceleration
angM = [cos(psi)*sin(theta)*cos(phi) + sin(psi)*sin(phi);
      sin(psi)*sin(theta)*cos(phi) - cos(psi)*sin(phi);
      cos(theta)*cos(phi)];

Ta = T/m * angM;

% Drag
Aa = -1/m .* A .* rdot;

Sphi = sin(phi); Stheta = sin(theta); Spsi = sin(psi);
Cphi = cos(phi); Ctheta = cos(theta); Cpsi = cos(psi);

C11 = 0;
C12 = (Iyy - Izz) * (dtheta*Cphi*Sphi + dpsi*Sphi^2*Ctheta) + (Izz - Iyy)*dpsi*Cphi^2*Ctheta - Ixx*dpsi*Ctheta;
C13 = (Izz - Iyy) * dpsi*Cphi*Sphi*Ctheta^2;
C21 = (Izz - Iyy) * (dtheta*Cphi*Sphi + dpsi*Sphi*Ctheta) + (Iyy - Izz)*dpsi*Cphi^2*Ctheta - Ixx*dpsi*Ctheta;
C22 = (Izz - Iyy) * dphi*Cphi*Sphi;
C23 = -Ixx * dpsi * Stheta * Ctheta + Iyy * dpsi * Sphi^2 * Stheta * Ctheta + Izz * dpsi * Cphi^2 * Stheta * Ctheta;
C31 = (Iyy - Izz) * dpsi * Ctheta^2 * Sphi * Cphi - Ixx * dtheta * Ctheta;
C32 = (Izz - Iyy) * (dtheta * Cphi * Sphi * Stheta + dphi * Sphi^2 * Ctheta) + (Iyy - Izz) * dphi * Cphi^2 * Ctheta...
        + Ixx * dpsi * Stheta * Ctheta - Iyy * dpsi * Sphi^2 * Stheta * Ctheta - Izz * psi * Cphi^2 * Stheta * Ctheta;
C33 = (Iyy - Izz) * dphi * Cphi * Sphi * Ctheta^2 - Iyy * dtheta * Sphi^2 * Ctheta * Stheta ...
        - Izz * dtheta * Cphi^2 * Ctheta * Stheta + Ixx * dtheta * Ctheta * Stheta;

C = [C11 C12 C13; C21 C22 C23; C31 C32 C33];    

J11 = Ixx; J12 = 0; J13 = -Ixx*Stheta;
J21 = 0;
J22 = Iyy * Cphi^2 + Izz * Sphi^2;
J23 = (Iyy - Izz) * Cphi * Sphi * Ctheta;
J31 = -Ixx * Stheta;
J32 = (Iyy - Izz)*Cphi*Sphi*Ctheta;
J33 = Ixx * Stheta^2 + Iyy * Sphi^2 * Ctheta^2 + Izz * Cphi^2*Ctheta^2;
J = [J11 J12 J13; J21 J22 J23; J31 J32 J33];  

z = [r;rdot;eta;etadot];

u = [T;taub];
f = [rdot;
     -1/m .* A.* rdot + T/m * angM - [0;0;g];
     etadot;
     J\(taub - C*etadot)];

A13 = zeros(12,3);
A2 = [eye(3);-(1/m).*A.*eye(3);zeros(6,3)];
A4 = [zeros(6,3);eye(3);-J\C];
Ac = [A13, A2, A13, A4];

B1 = [zeros(3,1); 1/m .* angM; zeros(6,1)];
B2 = [zeros(9,3); inv(J)];
Bc = [B1, B2];

Gc = [zeros(5,1); -g; zeros(6,1)];

Amat = jacobian(f,z);
Bmat = jacobian(f,u);

%Write the non-linear dynamics to a file
matlabFunction(f ,'file','getDynSys.m','vars',{'x', 'y', 'z', 'dx', 'dy', 'dz', 'phi', 'theta', 'psi', 'dphi', 'dtheta', 'dpsi', 'T', 'tauphi', 'tautheta', 'taupsi', 'm', 'g', 'Ixx', 'Iyy', 'Izz', 'A'},'outputs',{'dstates'});

%Write the non-linear dynamical matrices to a file
matlabFunction(Ac, Bc, Gc,'file','getDynMat.m','vars',{'phi', 'theta', 'psi', 'dphi', 'dtheta', 'dpsi', 'm', 'g', 'Ixx', 'Iyy', 'Izz', 'A'},'outputs',{'Ac','Bc','Gc'});

%Write the linearized dynamics to a file
matlabFunction(Amat,Bmat,'file','getLinSys.m','vars',{'phi', 'theta', 'psi', 'dphi', 'dtheta', 'dpsi', 'T', 'tauphi', 'tautheta', 'taupsi', 'm', 'Ixx', 'Iyy', 'Izz', 'A'},'outputs',{'Amat','Bmat'});


