A = [1,2,3;
     4,5,6;
     7,8,9];
 
x = [3;4;5];

deltax = 1e-3;

J = zeros(3);

for ii = 1:3
    xm = x;
    xp = x;
    xm(ii) = xm(ii) - deltax;
    xp(ii) = xm(ii) + deltax;
    J(:,ii) = (A * xm  - 2 * A * x + A * xp)/deltax;
end