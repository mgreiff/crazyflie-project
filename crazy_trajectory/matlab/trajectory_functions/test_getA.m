% N = 4
p = [5, 4, 3, 2, 1];
pol = @(t) p(1)*t^0 + p(2)*t^1 + p(3)*t^2 + p(4)*t^3 + p(5)*t^4;
dpol = @(t) p(2) + 2*p(3)*t + 3*p(4)*t^2 + 4*p(5)*t^3;
ddpol = @(t) 2*p(3) + 2*3*p(4)*t + 3*4*p(5)*t^2;
dddpol = @(t) 2*3*p(4) + 2*3*4*p(5)*t;
ddddpol = @(t) 2*3*4*p(5);
N = 4;
T = 1;
[A0, AT] = get_A(N, T);

disp((A0*p')')
disp([pol(0), dpol(0), ddpol(0), dddpol(0), ddddpol(0)])

disp((AT*p')')
disp([pol(T), dpol(T), ddpol(T), dddpol(T), ddddpol(T)])