load('rotor_data.mat')
fprintf(['Loading example data generated from a run with the continuous\n',...
         'parameters: L: 6 ,R: 5 ,Ke: 4 ,Kt: 4 ,J: 2 ,b: 3 ...\n'])
      
% LS identification of coefficients of the u->i pulse TF
Oi = [-I(2:end-1),-I(1:end-2),U(2:end-1),U(1:end-2)];
Yi = I(3:end);
thetai = (Oi'*Oi)\(Oi'*Yi);

% LS identification of coefficients of the omega->i pulse TF
Ow = [-omega(2:end-1),-omega(1:end-2),U(2:end-1),U(1:end-2)];
Yw = omega(3:end);
thetaw = (Ow'*Ow)\(Ow'*Yw);

% Solving of the linear system
b1w = thetaw(3);
b0w = thetaw(4);
b1i = thetai(3);
b0i = thetai(4);
a1 = thetai(1);
a0 = thetai(2);
b11 = b1w;
b21 = b1i;

k1 = (2*a1 + b0w/b11 - b0i);
k0 = a0 - (b0w/b11)*(b0i - a1);

a22 = (a0*b11*b21+b0w*b0i-b0w*a1*b21)/(b0w*b21-b0i*b11);
a11 = -a1 - a22;
a21 = (b0i - (a1 + a22)*b21)/b11;
a12 = -(a0 + (a1 + a22)*a22)/a21;

% Validate LS estimation
A = [a11, a12; a21, a22];
B = [b11; b21];

x = zeros(2,length(U));
for ii = 2:length(U)
    x(:,ii) = A*x(:,ii-1) + B*U(ii-1);
end

tt = linspace(0,h*(length(U)-1),length(U));
figure(1)
subplot(2,1,1)
plot(tt,U,'b-x')
title('Measured applied voltage as a function of time')
xlabel('Time [s]')
ylabel('Voltage, U_k [V]')

subplot(2,1,2)
hold on;
plot(tt,omega,'rx', 'markersize', 2)
plot(tt,I,'bx', 'markersize', 2)
plot(tt,x(1,:),'k')
plot(tt,x(2,:),'g')
xlabel('Time [s]')
ylabel('Angular speed, \omega_k [rad/s], Current, i_k [A]')
title('Measured and reconstrucated states as a function of time')
legend('\omega_k (measured)','i_k (measured)', '\omega_k (reconstructed)','i_k (reconstructed)')

% Reconstructs original system
C = eye(2);
syshatd = ss(A,B,C,[],h);
syshatc = d2c(syshatd);

Ahatc = syshatc.A;
Bhatc = syshatc.B;

fprintf('\nEstimated parameters:\n')
param.res = syshatc.B(1,1);
param.L = 1/syshatc.B(2,1);
param.R = -param.L*Ahatc(2,2);
param.Ke = -param.L*Ahatc(2,1);
param.Kt = param.Ke;
param.J = param.Kt/Ahatc(1,2);
param.b = -param.J*Ahatc(1,1)