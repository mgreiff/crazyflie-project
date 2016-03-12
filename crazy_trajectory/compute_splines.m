function [ P ] = compute_splines( points , times , N , cost )
    
    nS = length(points(1,:))-1;
    T = diff(times);

    if 2*length(points(:,1)) > N
        error(['Warning in compute_splines. Optimization will fail due'+...
               'to low polynomial order.'])
    end
    
    if length(cost) ~= N+1
         error('Warning in compute_splines. Too few elements in cost array.')
    end
    
    % Constructs the hessian
    Q = zeros((N+1)*nS,(N+1)*nS);
    for ii = 1:nS
        Qhat = get_Q( N , T(ii) , cost);
        index = (N+1)*(ii-1)+1:(N+1)*ii;
        Q(index,index) = Qhat;
    end
    
    A = zeros(nS*3*2,nS*(N+1)); % 6xN for each spline
    b = zeros(nS*3*2,1);
    for ii  = 1:nS
        boundaryPoints = [points(1:3,ii);points(1:3,ii+1)];

        [A0, AT] = get_A( N , T(ii));
        
        Ap = [A0(1:3,:);AT(1:3,:)];

        for jj = 1:6
            disp(size(Ap))
        	if ~isnan(boundaryPoints(jj))
                b(6*(ii-1) + jj) = boundaryPoints(jj);
            else
                Ap(jj,:) = 0;
        	end
        end
        A((ii-1)*6+1:ii*6, (ii-1)*(N+1)+1:ii*(N+1)) = Ap;
    end

    A(b==0,:)=[];
    b(b==0)=[];

    [ P, ~ ] = quadprog(Q,zeros(length(Q),1),[],[],A,b);
end

