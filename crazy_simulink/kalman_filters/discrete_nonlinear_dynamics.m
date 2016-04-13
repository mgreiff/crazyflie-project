function xnew = discrete_nonlinear_dynamics(omega, x, g, m, k, A, I, l, b, Ts)
    % Equations from Teppo Luukkonen, Modelling and control of quadcopter
    phi = x(7); theta = x(8); psi = x(9); % Roll, pitch, yaw
    phidot = x(10); thetadot = x(11); psidot = x(12); % Roll, pitch, yaw
    
    T = k * sum(omega.^2); % (7)

    % Define the C-matrix as (19)
    Ixx = I(1); Iyy = I(2); Izz = I(3);
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
    
    % (8)
    tau_phi =  l * k * (-omega(2)^2 + omega(4)^2);
    tau_theta =  l * k * (-omega(1)^2 + omega(3)^2);
    tau_psi =  -b * sum(omega(1)^2 - omega(2)^2 + omega(3)^2 - omega(4)^2); % Modified from the text, rotor 2 and 4 spin in oposite direction
    tau_b = [tau_phi; tau_theta; tau_psi];
    
    % (16)
    J11 = Ixx; J12 = 0; J13 = -Ixx*Stheta;
    J21 = 0;
    J22 = Iyy * Cphi^2 + Izz * Sphi^2;
    J23 = (Iyy - Izz) * Cphi * Sphi * Ctheta;
    J31 = -Ixx * Stheta;
    J32 = (Iyy - Izz)*Cphi*Sphi*Ctheta;
    J33 = Ixx * Stheta^2 + Iyy * Sphi^2 * Ctheta^2 + Izz * Cphi^2*Ctheta^2;
    J = [J11 J12 J13; J21 J22 J23; J31 J32 J33];  

    invJ = inv(J);
    
    % Sets up continuous time system
    I3 = eye(3);

    Ac = zeros(12);
    Ac(1:3,4:6) = I3;
    Ac(4:6,4:6) = -(1/m)*diag(A);
    Ac(7:9,10:12) = I3;
    Ac(10:12,10:12) = -invJ*C;

    Rz = (1/m) * [Cpsi*Stheta*Cphi+Spsi*Sphi;...
                  Spsi*Stheta*Cphi-Cpsi*Sphi;...
                  Ctheta*Cphi];

    Bc = zeros(12,4);
    Bc(4:6,1) = Rz;
    Bc(10:12,2:4) = invJ;

    Cc = eye(12);

    Dc = [];
    sysc = ss(Ac,Bc,Cc,Dc);
    sysd = c2d(sysc, Ts);
    
    u = [T;tau_b];
    
    Gd = zeros(12,1);
    Gd(6) = -g*Ts;
    
    xnew = sysd.a*x + sysd.b*u + Gd;
end