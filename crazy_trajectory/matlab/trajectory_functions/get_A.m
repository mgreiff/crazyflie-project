function [ A0 , AT ] = get_A( N , T )
    % Computes the constraint matrix at time t=0 (A0) and t=T (AT), together
    % [A0;AT] forms A_i, which constitutes the constraint matrix for
    % Spline p_i, such that A_i*p_i-b_i=0. Returns A0 and AT separate, so
    % that the enpoints can be treated differently (neuman/dirichlet
    % conditions at differering derivatives).
    A0 = zeros(N+1,N+1);
    AT = zeros(N+1,N+1);

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
            if n >= r-1
                AT(row, col) = T^(n - r);
                for m = 0:r-1
                    AT(row, col) = AT(row, col)*(n - m);
                end
            end
        end
    end
end

