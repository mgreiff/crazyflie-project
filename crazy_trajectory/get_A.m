function [ A0 , AT ] = get_A( N , T )
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
            if n >= r
                AT(row, col) = T^(n-r);
                for m = 0:r-1
                    AT(row, col) = AT(row, col)*(r - m);
                end
            end
        end
    end
end

