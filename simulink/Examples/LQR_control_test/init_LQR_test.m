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
modified_sys = minreal(sys);
Q = eye(size(modified_sys.A));
Q(1,1) = 100;
Q(2,2) = 0;
Q(3,3) = 100;
Q(4,4) = 0;
Q(5,5) = 100;
Q(6,6) = 0;
Q(7,7) = 100;
Q(8,8) = 0;
R = 10*eye(length(modified_sys.B(1,:)));
[K,S,E] = lqr(modified_sys,Q,R);