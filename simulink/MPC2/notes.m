T = 6/2.5;
im = 4/2.5;

s = tf('s');
Gor = (T^2+im^2)/((s+T+im*i)*(s+T-im*i));
[num,den] = tfdata(Gor, 'v');
%[A B C D] = tf2ss(num, den)

step(Gor, 3); %Approximate tf of angle-pd-controller

KphiD = KetaD(1);
KphiP = KetaP(1);

Gang = tf([KphiP],[1 KphiD KphiP]);