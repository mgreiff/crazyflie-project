h = 0.0001; Tf = 2;
t = 0:h:Tf;

t1 = t(t<=0.5);
t2 = t(0.5<t & t<=1);
t3 = t(1<t & t<=1.5);
t4 = t(1.5<t & t<=2);

w1 = 2*sin(t1*4*pi);
w2 = 2*sin(t1*4*pi);
w3 = 2*sin(t1*4*pi);
w4 = 2*sin(t1*4*pi);

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

w = hover_omega+30*[w1;w2;w3;w4];

plot(t, w);

M = [t;w];
save('example_omega.mat', 'M');