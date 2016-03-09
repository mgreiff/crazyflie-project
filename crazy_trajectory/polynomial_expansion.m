% See http://michigancmes.org/papers/roy7.pdf

% Points to pass through
p1 = [1;2;0;0]; % pos, vel, ac, jerk
p2 = [3;-1;0;0];
p3 = [2;1;0;0];
times = [0,5,8];
points = [p1,p2,p3];

nSplines = length(points(1,:))-1;
rMax = 3;
dimQr = nSplines;
dimQ = (rMax + 1) * nSplines;
Q = zeros(dimQ,dimQ);

T = 5; % Sets all times to 5 for now

for r = 0:rMax
    Qr = zeros(dimQr,dimQr);
    for ii = 0:dimQr-1
        for ll = 0:dimQr-1
            if ii >= r && ll >= r
                Qr(ii+1,ll+1) = 2*(T.^(ii+ll-2*r+1))/(ii+ll-2*r+1);
                for m = 0:r-1
                    Qr(ii+1,ll+1) = Qr(ii+1,ll+1)*(ii-m)*(ll-m);
                end
            end
        end
    end
    subIs = r*dimQr + 1;
    subIf = (r+1)*dimQr;
    Q(subIs:subIf,subIs:subIf) = Qr;
end

A0 = zeros((rMax+1)*nSplines,(rMax+1)*nSplines);
AT = zeros((rMax+1)*nSplines,(rMax+1)*nSplines);

for r = 0:rMax
    for s = 1:nSplines
        for n = 0:rMax
            row = r*2 + s;
            col = s + nSplines*n;
            %A0(row, col) = r+1;
            if r == n
                A0(row, col) = 1;
                for m = 0:r-1
                     A0(row, col) = A0(row, col)*(r - m);
                end
            end
            if r <= n
                AT(row, col) = T^(n-r);
                for m = 0:r-1
                    AT(row, col) = AT(row, col)*(r - m);
                end
            end
        end
    end
end

A = [A0;AT];

b0 = [p1,p2];
bT = [p2,p3];

b = [reshape(b0',[numel(b0),1]);
     reshape(bT',[numel(bT),1])];

P = quadprog(Q,zeros(length(Q),1),A,b);

if 1
    plot(times,points(1,:),'x')
    hold on;
    tt1 = linspace(0,5,100);
    p1 = @(x) P(1) + P(3)*x + P(5)*x.^2 + P(7)*x.^3;
    plot(tt1,p1(tt1))
    p2 = @(x) P(2) + P(4)*x + P(6)*x.^2 + P(8)*x.^3;
    tt2 = linspace(5,10,100);
    plot(tt2,p2(tt2))
end