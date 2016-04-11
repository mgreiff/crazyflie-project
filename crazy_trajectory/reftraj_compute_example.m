% Polynomial order
N = 5;

% Minimum snap cost vector
cost = [0,0,0,1,0,0,0,0,0,0];

% Sets all conditions free
a1 = nan(N+1,3);
a2 = nan(N+1,3);
a3 = nan(N+1,3);
a4 = nan(N+1,3);

% Enforces positional conditions
a1(1,:) = 5; % x,y,z-positions in the first point
a1(2,1) = 0; % x-velocity in the first point

a2(1,1) = 7; % x-position in the second point
a2(1,2) = 6; % y-position in the second point
a2(1,3) = 8; % z-position in the second point

a3(1,1) = 0; % x-position in the third point
a3(1,2) = 10; % y-position in the third point
a3(1,3) = 5; % z-position in the third point

a4(1,1) = 3; % x-position in the fourth point
a4(1,2) = 5; % y-position in the fourth point
a4(1,3) = 0; % z-position in the fourth point

times = 8*[1,1.7,3,4];

coeff = [];
for ii = 1:3
    points = [a1(:,ii),a2(:,ii),a3(:,ii),a4(:,ii)];
    P = compute_splines( points , times , N , cost );
    coeff = [coeff, P];
end
splines.coeff = coeff;
splines.times = times;
splines.N = N;
save('trajectoryData', 'splines')