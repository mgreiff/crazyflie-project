h = 0.0001; Tf = 2.5;
t = 0:h:Tf;
delta_omega = 8;

t1 = t(t<=0.5);
t2 = t(0.5<t & t<=1);
t3 = t(1<t & t<=1.5);
t4 = t(1.5<t & t<=2);
t5 = t(2<t & t<=2.5);

w1 = 10*sin(t1*4*pi);
w2 = 10*sin(t1*4*pi);
w3 = 10*sin(t1*4*pi);
w4 = 10*sin(t1*4*pi);

w2 = [w2 -sin(t2*4*pi)];
w4 = [w4 sin(t2*4*pi)];
w1 = [w1 0*t2];
w3 = [w3 0*t2];

w1 = [w1 -sin(t3*4*pi)];
w3 = [w3 sin(t3*4*pi)];
w2 = [w2 0*t3];
w4 = [w4 0*t3];

w1 = [w1 -sin(t4*4*pi)];
w3 = [w3 -sin(t4*4*pi)];
w2 = [w2 sin(t4*4*pi)];
w4 = [w4 sin(t4*4*pi)];

w1 = [w1 0.*t5];
w3 = [w3 0.*t5];
w2 = [w2 0.*t5];
w4 = [w4 0.*t5];

w = hover_omega+delta_omega*[w1;w2;w3;w4];

M = [t;w];
save('example_omega.mat', 'M');