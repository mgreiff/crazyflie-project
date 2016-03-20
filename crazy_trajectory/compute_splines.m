function [ P ] = compute_splines( points , times , N , cost )
    % Computes the polynomial coefficients given a set of points, the times
    % at which the points should be passed, the maximum derivative and the
    % cost array.
    %
    % ARGS:
    %     points - An array containing the points where a dirichlet
    %         condition is enforced by a numeric value and a Neumann
    %         condition is enforced if the element is set to NaN. The first
    %         row corresponds to the position, the second to velocities and
    %         the Nth row correspons to values of the Nth derivative. The
    %         points are passed though in the order of the columns, such
    %         that the point in column 1 is passed first, then the point in
    %         column 2 and so on. (array N+1xS) where S denoted the number
    %         ot splines.
    %     times - the time at which a point should be passed, each element
    %         corresponds to a row in "points" and the array has to be
    %         monotonicaly increasing.
    %     N - The highest occuring derivative in the polynomial (int > 1).
    %     cost - The cost vector (array 1xN+1).
    
    % RETURNS:
    %     P  - The polynomial coefficients (array (N+1)*Sx1).
    %         [p_0, p_1, ..., p_N, p_0, p_1, ..., p_N, ...]'
    %          ---of spline 1---    ---of spline 2---
    
    
    nS = length(points(1,:))-1;
    T = diff(times);
    
    % Constructs the hessian
    Q = zeros((N+1)*nS,(N+1)*nS);
    for ii = 1:nS
        Qhat = get_Q( N , T(ii) , cost);
        index = (N+1)*(ii-1)+1:(N+1)*ii;
        Q(index,index) = Qhat;
    end
    
    Afree = zeros((N+1)*(nS+1),(N+1)*nS);
    Abound = [];
    bbound = [];
    for ii = 1:nS
        % Creates a large matrix of free variable constraints
        [ A0 , AT ] = get_A( N , T(ii) );
        rows = (ii-1)*(N+1)+1:(ii+1)*(N+1);
        cols = (ii-1)*(N+1)+1:ii*(N+1);
        Afree(rows,cols) = [A0;-AT];
        
        % Sets fixed consraints
        pp = [points(:,ii);points(:,ii+1)];
        Ap = [A0;AT];
        for jj = 1:length(pp)
            if ~isnan(pp(jj))
                Arow = zeros(1,(N+1)*nS);
                Arow(cols) = Ap(jj,:);
                Abound = [Abound; Arow];
                bbound = [bbound; pp(jj)];
            end
        end
    end

    Afree(1:N+1,:) = [];
    Afree(end-N:end,:) = [];
    bfree = zeros(length(Afree(:,1)),1);
    A = [Abound;Afree];
    b = [bbound;bfree];
    
    [ P, ~ ] = quadprog(Q,zeros(length(Q),1),[],[],A,b);
end

