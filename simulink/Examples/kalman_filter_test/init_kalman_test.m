KF_P0 = eye(12); % TBD experimentally
KF_Q = eye(12);
KF_R = eye(7);  % TBD experimentally
KF_R(1,1) = 0.1;
KF_R(2:4,2:4) = 0.001*eye(3);
KF_R(5:7,5:7) = 0.001*eye(3);