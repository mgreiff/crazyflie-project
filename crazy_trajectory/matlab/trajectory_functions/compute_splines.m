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
        throw(MException(msgID, msgtext))
    end

    % Constructs the hessian
    Q = zeros((N+1)*nS,(N+1)*nS);
    for ii = 1:nS
        Qhat = get_Q( N , T(ii) , cost);
        index = (N+1)*(ii-1)+1:(N+1)*ii;
        Q(index,index) = Qhat;
    end
    Abound = [];
    bbound = [];
    AfreeL = [];
    AfreeH = [];

    for ii = 1:nS

        [ A0 , AT ] = get_A( N , T(ii) );
        
        % Iterates over the end point conditions in the starting point
        % (there is no need to) look at the terminal point assuming the
        % input data passes the consistency checks
        for jj = 1:N+1
            % Treats end point conditions in the starting point
            epc = points(jj,ii);
            if ~isnan(epc)
                if ~isinf(epc)
                    % Bound end point conditions
                    disp(['bound', num2str(epc)])
                    Abound = [Abound; A0(jj,:)];
                    bbound = [bbound; epc];
                elseif ii ~= nS
                    % Free end point conditions
                    AfreeL = [AfreeL; A0(jj,:)]; 
                    disp(['free', num2str(jj)])
                end
            end
            
            % Treats end point conditions in the terminal point
            epc = points(jj,ii+1);
            if ~isnan(epc) && ii == nS
                if ~isinf(epc)
                    % Bound end point conditions in terminal point
                    disp(['bound', num2str(epc)])
                    Abound = [Abound; AT(jj,:)];
                    bbound = [bbound; epc];
                elseif ii ~= 1
                    % Free end point conditions
                    AfreeH = [AfreeH; AT(jj,:)]; 
                    disp(['free', num2str(jj)])
                end
            end
        end
    end

    % Sets terminal point conditions of the last spline
    Afree = [AfreeL, -AfreeH];
    if isempty(Afree)
        bfree = [];
    else
        bfree = zeros(size(Afree,1),1);
    end
    
    A = [Abound; Afree];
    b = [bbound; bfree];

    options = optimoptions('quadprog');
    options.TolCon = 1e-12;
    options.TolFun = 1e-12;
    options.Diagnostics = 'off';
    %options.Algorithm = 'active-set';
    options.Algorithm = 'interior-point-convex';
    [P, ~, EXITFLAG, OUTPUT, LAMBDA] = quadprog(Q,zeros(length(Q),1),[],[],A,b,[],[],[],options);
end

