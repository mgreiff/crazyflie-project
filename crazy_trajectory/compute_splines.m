function [ P ] = compute_splines( points , times , N , cost )
    
    nS = length(points(1,:))-1;
    T = diff(times);

    if 2*length(points(:,1)) < N
        error(['Warning in compute_splines. Optimization will fail due'+...
               'to low polynomial order.'])
    end
    
    if length(cost) ~= N+1
         error('Warning in compute_splines. Too few elements in cost array.')
    end
    
    % Constructs the hessian
    Q = zeros(N+1,N+1);

    for r = 0:N
        Qr = zeros(N+1,N+1);
        for ii = 0:N
            for ll = 0:N
                if ii >= r && ll >= r
                    row = ii + 1;
                    col = ll + 1;
                    Qr(row,col) = 2*(T.^(ii+ll-2*r+1))/(ii+ll-2*r+1);
                    for m = 0:r-1
                        Qr(row,col) = Qr(row,col)*(ii-m)*(ll-m);
                    end
                end
            end
        end
        Q = Q + cost(r+1).*Qr;
    end

    % Constructs the constraints
    A0 = zeros((N+1)*nS,(N+1)*nS);
    AT = zeros((N+1)*nS,(N+1)*nS);

    for r = 0:N
        for n = 0:N
            row = r+1;
            col = n+1;
            if r == n
                A0(row, col) = 1;
                for m = 0:r-1
                     A0(row, col) = A0(row, col)*(r - m);
                end
            end
            if n >= r
                AT(row, col) = T^(n-r);
                for m = 0:r-1
                    AT(row, col) = AT(row, col)*(r - m);
                end
            end
        end
    end

    a1 = points(:,1);
    a2 = points(:,2);

    A = [A0(1:2,:); AT(1,:)];
    b0 = a1;
    bT = a2;

    b = [b0(1:2); bT(1)];

    [ P, ~ ] = quadprog(Q,zeros(length(Q),1),[],[],A,b);
end

