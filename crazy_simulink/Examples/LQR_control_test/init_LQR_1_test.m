%% Set up linearized system model and compute LQR gain K

Ad = diag(A);
a1 = 4.8; a2 = 8.32; b = 8.32;
a1 = KetaD(1); a2 = KetaP(1); b = KetaP(1);

T0 = m*g;

Am1 = [zeros(10,3)];
Am2 = [eye(3); -Ad/m; zeros(4,3)];
Am3 = [zeros(3,2); T0/m 0; 0 -T0/m; zeros(3,2); -a2*eye(2)];
Am4 = [zeros(6,2); eye(2); -a1*eye(2)];
Am = [Am1 Am2 Am3 Am4];

Bm = zeros(10, 3);
Bm(9, 1) = b;
Bm(10, 2) = b;
Bm(6, 3) = 1/m;

Cm = eye(10);

Dm = zeros(10, 3);

sys = ss(Am,Bm,Cm,Dm,0.1);
disp(['Controllable states: ',rank(ctrb(sys))])
disp(['Observable states: ',rank(obsv(sys))])
disp(['Eigenvalues of A: ', mat2str(eig(sys.a))])


[K,S,E] = lqr(sys,Q,R);