if 1 % Use full sym
    syms Ixx Iyy Izz psi theta phi phidot thetadot psidot
end
if 0 % Use partial sym
    syms psi theta phi
    phidot = 0;
    thetadot = 0;
    psidot = 0;
    Ixx = I(1); Iyy = I(2); Izz = I(3);
end
Sphi = sin(phi); Stheta = sin(theta); Spsi = sin(psi);
Cphi = cos(phi); Ctheta = cos(theta); Cpsi = cos(psi);

C11 = 0;
C12 = (Iyy - Izz) * (thetadot*Cphi*Sphi + psidot*Sphi^2*Ctheta) + (Izz - Iyy)*psidot*Cphi^2*Ctheta - Ixx*psidot*Ctheta;
C13 = (Izz - Iyy) * psidot*Cphi*Sphi*Ctheta^2;
C21 = (Izz - Iyy) * (thetadot*Cphi*Sphi + psidot*Sphi*Ctheta) + (Iyy - Izz)*psidot*Cphi^2*Ctheta - Ixx*psidot*Ctheta;
C22 = (Izz - Iyy) * phidot*Cphi*Sphi;
C23 = -Ixx * psidot * Stheta * Ctheta + Iyy * psidot * Sphi^2 * Stheta * Ctheta + Izz * psidot * Cphi^2 * Stheta * Ctheta;
C31 = (Iyy - Izz) * psidot * Ctheta^2 * Sphi * Cphi - Ixx * thetadot * Ctheta;
C32 = (Izz - Iyy) * (thetadot * Cphi * Sphi * Stheta + phidot * Sphi^2 * Ctheta) + (Iyy - Izz) * phidot * Cphi^2 * Ctheta...
        + Ixx * psidot * Stheta * Ctheta - Iyy * psidot * Sphi^2 * Stheta * Ctheta - Izz * psi * Cphi^2 * Stheta * Ctheta;
C33 = (Iyy - Izz) * phidot * Cphi * Sphi * Ctheta^2 - Iyy * thetadot * Sphi^2 * Ctheta * Stheta ...
        - Izz * thetadot * Cphi^2 * Ctheta * Stheta + Ixx * thetadot * Ctheta * Stheta;
C = [C11 C12 C13; C21 C22 C23; C31 C32 C33];

J11 = Ixx; J12 = 0; J13 = -Ixx*Stheta;
J21 = 0;
J22 = Iyy * Cphi^2 + Izz * Sphi^2;
J23 = (Iyy - Izz) * Cphi * Sphi * Ctheta;
J31 = -Ixx * Stheta;
J32 = (Iyy - Izz)*Cphi*Sphi*Ctheta;
J33 = Ixx * Stheta^2 + Iyy * Sphi^2 * Ctheta^2 + Izz * Cphi^2*Ctheta^2;
J = [J11 J12 J13; J21 J22 J23; J31 J32 J33];

etastate = [psi theta phi phidot thetadot psidot];
NonlinearPart = [];
iJC = linsolve(inv(J),C)*[phidot thetadot psidot]';%linsolve(inv(J,))
%iJCvectorvaluedFunction = sum(iJC')';
for jj = 1:6
    NonlinearPart = [NonlinearPart diff(iJC,etastate(1,jj))];
end

% The idea is to save the expression for the nonlinear part so it has
% to be generated only once and the evaluated on every iteration

% Some linearisation point
psi = 0;
theta  = 0;
phi = 0;
phidot = 0;
thetadot = 0;
psidot = 0;

tic
nlp = eval(NonlinearPart);
toc
% 
% disp('Complete system matrix...')
% Anonlin = [zeros(9,12);zeros(3,6),nlp];
% 
% Alin = zeros(12);
% Alin(1:3,1:3) = eye(3);
% Alin(4:6,4:6) = -(1/m).*diag(A);
% Alin(7:9,7:9) = eye(3);
% 
% Afull = Alin + Anonlin;
% disp(Afull)