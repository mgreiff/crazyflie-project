
syms Ixx Iyy Izz k l b g m omega1 omega2 omega3 omega4 real

B1 = zeros(9,4);
B1(6,1)=1;
B2 = [zeros(3,1),diag([1/Ixx,1/Iyy,1/Izz])];
Bc = [B1;B2];

M = [k k k k; 0 -k*l 0 k*l; -k*l 0 k*l 0; -b b -b b];
omegavec = [omega1 omega2 omega3 omega4];
NonlinearPart = [];
vvFunction = Bc*M*omegavec'.^2;
for jj = 1:4
    NonlinearPart = [NonlinearPart diff(vvFunction,omegavec(1,jj))];
end

omega1 = sqrt(g*m/(4*k));
omega2 = sqrt(g*m/(4*k));
omega3 = sqrt(g*m/(4*k));
omega4 = sqrt(g*m/(4*k));

eval(NonlinearPart)