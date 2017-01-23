function [ P ] = compute_splines( points , times , N , cost )
    %% function [ P ] = compute_splines( points , times , N , cost )
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
    %         column 2 and so on. (array N+1xS) where S denotes the number
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
    
    %% Input data check 
    msgID = 'myComponent:inputError';
    msgtext = 0;
    for ii = 1:nS
        Nc = sum(sum(~isnan(points(:,ii:ii+1))));
        if Nc > N
            msgtext = ['Warning. Cannot create spline (number ',...
                       num2str(ii), '), the number of endpoint'...
                       ' conditions (Nc = ', num2str(Nc), ') exceeds',...
                       ' the polynomial order (N = ', num2str(N), ').'];
        end
    end
    if length(cost) ~= N + 1
        msgtext = ['Warning. Length of the cost vector must be N + 1, '...
                   'but is len(cost) = ', num2str(length(cost))];
    elseif length(times) ~= length(points(1,:))
        msgtext = ['Warning. Length of the time vector must be equal '...
                   'to the number of points.'];
    elseif ~isempty(find(T <= 0, 1))
        msgtext = ['Warning. The time vector must be strictly '... 
                   'monotonically increasing, which is not the case at '...
                   ' the indices ', num2str(find(T <=0))];
    end
    if msgtext ~= 0
        %throw(MException(msgID, msgtext))
    end

    %% Constructs the hessian
    Q = zeros((N+1)*nS,(N+1)*nS);
    for ii = 1:nS
        Qhat = get_Q( N , T(ii) , cost);
        index = (N+1)*(ii-1)+1:(N+1)*ii;
        Q(index,index) = Qhat;
    end

    %% Constructs the full constraint matrices
    Abound = zeros((N + 1) * (nS + 1), (N + 1) * nS);
    Afree = zeros((N + 1) * (nS - 1), (N + 1) * nS);

    for ii = 1:nS
        [ A0 , AT ] = get_A( N, T(ii));

        % Sets up a large matrix of free and bound constraints
        dind = (ii - 1)*(N + 1) + 1;
        Abound(dind:dind+N,dind:dind+N) = A0;
        disp(dind)
        if ii == nS && nS > 1
            Abound(dind+N+1:dind+2*N+1,dind:dind+N) = AT;
            Afree(dind-N-1:dind-1,dind:dind+N) = -A0;
        elseif ii == 1 && nS > 1
            Afree(dind:dind+N,dind:dind+N) = AT;
        elseif nS > 1
            Afree(dind-N-1:dind-1,dind:dind+N) = -A0;
            Afree(dind:dind+N,dind:dind+N) = AT;
        end
    end
    
    ap = reshape(points, numel(points), 1);
    indices = isinf(ap) | isnan(ap);
    bbound = ap(~isinf(ap) & ~isnan(ap));
    Abound(indices,:) = [];
    
    interiorpoints = points(:,2:end-1);
    interiorpoints(1,:) = inf;
    interiorpoints(2,:) = inf;
    ip = reshape(interiorpoints, numel(interiorpoints), 1);
    indices = ~isinf(ip);
    Afree(indices, :) = [];
    bfree = zeros(size(Afree,1),1);
    
    A = [Abound; Afree];
    b = [bbound; bfree];

    options = optimoptions('quadprog');
    options.TolCon = 1e-12;
    options.TolFun = 1e-12;
    options.Diagnostics = 'off';
    options.Algorithm = 'interior-point-convex';
    [P, ~, ~, ~, ~] = quadprog(Q,zeros(length(Q),1),[],[],A,b,[],[],[],options);
end

