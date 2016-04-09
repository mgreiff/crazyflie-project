if 1 % Use full sym
    xd = sym('xd','real');
    yd = sym('yd','real');
    zd = sym('zd','real');
    x = sym('x','real');
    y = sym('y','real');
    z = sym('z','real');
    phi = sym('phi','real');
    theta = sym('theta','real');
    psi = sym('psi','real');
    phidot = sym('phidot','real');
    thetadot = sym('thetadot','real');
    psidot = sym('psidot','real');
    Ixx = sym('Ixx','real');
    Iyy = sym('Iyy','real');
    Izz = sym('Izz','real');
    d = sym('d','real');
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

iJC = linsolve(J,C);
A = [zeros(3),eye(3),zeros(3,6);
     zeros(3),d*eye(3),zeros(3,6);
     zeros(3,9),eye(3);
     zeros(3,9),iJC];
states = [x y z xd yd zd phi theta psi phidot thetadot psidot]';

NonlinearPart = [];
for jj = 1:12
    NonlinearPart = [NonlinearPart diff(A*states,states(jj))];
end

% The idea was to save the expression for the nonlinear part so it has
% to be generated only once and the evaluated on every iteration, but this
% proved to be costly, now the script is soley used to linearize the system
% matrix

d = -0.25;
Ixx = 0.0001;
Iyy = 0.0001;
Izz = 0.0002;

% Some linearisation point
psi = 0.1;
theta  = 0.1;
phi = 0.1;
phidot = 0.1;
thetadot = 0.1;
psidot = 0.1;

tic
linearizedA = eval(NonlinearPart);
toc

g = 9.81;
m = 0.468;      % kg
l = 0.225;      % m
k = 2.980e-6;   % unitless
b = 1.140e-7;   % unitless

Mw = [k,k,k,k;
      0,-k*l,0,k*l;
      -k*l,0,k*l,0
      -b,b,-b,b];
Bc = zeros(12,4);
Bc(6,1) = 1;
Bc(10:12,2:4) = eval(inv(J));

linearizedB = sqrt(g*m/k)*2.*Bc*Mw
           
disp('Rank of contollability matrix')
rank(ctrb(linearizedA,linearizedB)) %=8

% --> locally uncontrollable at linearization point when stable giving problems in LQR design

disp('Rank of observability matrix')
C1 = [eye(3),zeros(3,9);
      zeros(3,6),eye(3),zeros(3)];
C2 = [zeros(7,2) eye(7) zeros(7,3)];
rank(obsv(linearizedA,C1))
rank(obsv(linearizedA,C2))


