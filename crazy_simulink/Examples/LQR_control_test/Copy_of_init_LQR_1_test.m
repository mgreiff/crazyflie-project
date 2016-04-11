%% Set up linearized system model and compute LQR gain K

Ixx = I(1);
Iyy = I(2);
Izz = I(3);

Am = [zeros(3),eye(3),zeros(3,6);
      zeros(3),-diag(A)/m,zeros(3,6);
      zeros(3,9),eye(3);
      zeros(3,12)];

Bm = sqrt(g*m/k).*[zeros(5,4);
                   k,k,k,k;
                   zeros(3,4);
                   0,-k*l/Ixx,0,k*l/Ixx;
                   -k*l/Iyy,0,k*l/Iyy,0;
                   -b/Ixx,b/Ixx,-b/Ixx,b/Ixx];
Cm = eye(12);
sys = ss(Am,Bm,Cm,[]);
truncated_sys = minreal(sys);

Atrunc = truncated_sys.a;
Btrunc = truncated_sys.b;
Ctrunc = eye(8);

sys = ss(Atrunc,Btrunc,Ctrunc,[]);
% Regular LQR
Q = eye(8);
R = 10*eye(length(truncated_sys.b(1,:)));
[K,S,E] = lqr(sys,Q,R);
G1 = -inv(R)*Btrunc'*S;
Temp = inv((Atrunc+Btrunc*G1)');
G2 = -inv(R)*Btrunc'*Temp*Ctrunc'*Q;

% With integrator
% Aint = [zeros(size(truncated_sys.a)),eye(size(truncated_sys.a));
%         zeros(size(truncated_sys.a)),truncated_sys.a];
% Bint = [zeros(size(truncated_sys.b));
%         truncated_sys.b];
% Cint = [zeros(8),eye(8)];
% Dint = [];
% int_sys = ss(Aint,Bint,Cint,Dint);
% Q = eye(20,20);
% R = 10*eye(4);
%[K,S,E] = lqi(truncated_sys,Q,R); doesn't work?
%[K,S,E] = lqr(int_sys,Q,R); doesn't work?