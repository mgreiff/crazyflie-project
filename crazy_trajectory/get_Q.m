function Q = get_Q( N , T , cost)
    % Constructs the hessian matrix of a spline given its cost vector,
    % punishing derivatives of various orders differently.
    %
    % ARGS:
    %    N - The highest occuring derivative in the polynomial (int > 1).
    %    T - The time interval on which the spline is defined, i.e., if
    %    p(t)\in[ts,tf], then T = tf-ts (double).
    %    cost - The cost vector (array 1xN+1).
    %
    % RETURNS:
    %    Q - The hessian matrix of the spline (array N+1xN+1)

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
end

