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
Cm(4:5,:) = [];
Cm(1:2,:) = [];
sys = ss(Am,Bm,Cm,[]);
m_sys = minreal(sys);
Am = m_sys.A;
Bm = m_sys.B;
Cm = m_sys.C;
i_sys = ss([Am,zeros(size(Am));eye(size(Am)),zeros(size(Am))],[Bm;zeros(size(Bm))],[Cm,zeros(size(Cm))],[]);
Q = 0.01.*eye(size(i_sys.a));
Q(1,1) = 100000;
Q(2,2) = 0.001;
Q(3,3) = 10000;
Q(4,4) = 10000;
Q(5,5) = 10000;
Q(6,6) = 0.001;
Q(7,7) = 0.001;
Q(8,8) = 0.001;
R = 10*eye(length(i_sys.B(1,:)));
[K,S,E] = lqr(i_sys,Q,R);

LQR_K = 0.17;
LQR_I = 0;
LQR_D = 0.15;