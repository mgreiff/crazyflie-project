function [x, as, iter] = getfeasible(A,b)
%
% [x, as, iter] = getfeasible(A,b)
%
% finds a feasible vector that fulfills the linear inequality
% constraints:
%
%    A*x <= b
%
% Return arguments are:
% 
%    x    A feasible vector
%    as   The set of active constraints (may be empty)
%    iter The number of iterations
%
% The algorithm is based on Fletcher (1987) p. 166.
%
% See also: qp_as
%

% Idea: Introduce a slack variable s, and find an s such that
% -5 <= s <= -1e-5 subject to Ax+s <= b
%
% Based on Fletcher (1987) p 166

verbosity = 0;

if verbosity,
  global Aa_index_history;
  Aa_index_history=[];
end


[m,n]=size(A);

mc = max(b);

A = [[A; zeros(2,n)] [-ones(m,1);-1; 1]];
b = [b;5; -1e-5];

[m,n]=size(A);

% Book keeping vectors
x = [zeros(n-1,1); mc+1];
x = zeros(n,1);

%x = randn(n,1);

%load x_fea_init

%save x_fea_init x

l = zeros(n,1); % The Lagrange multipliers
Aa = eye(n); % Active set A matrix
S = eye(n);
as = zeros(m,1); % Indicates which constraints are active 1 for
                 % active 0 for not in the active set
Aa_index = zeros(n,1); % Zero if the constraint is a pseudo
                      % constraint, otherwise the constraint number
g = zeros(n,1); % Gradient of cost function
fea = 0; % 1 if a feasible point is found. 0 otherwise
iter = 0;
nbr_small_alpha = 0;
time_elapsed = 0;
feas_update = 1;


cycling_detector = zeros(5,5); % A toeplitz matrix used to detect cycling

while ~fea
  
  if verbosity,
    tic
  end

  % Update only if there has been a change in the active set
  % Feasiblization of the current solution, makes sure that x is
  % really fullfilling the equality constraints.
  if (feas_update==1),

    Aqwe=A(find(as>0),find(Aa_index>0));
    bqwe = b(find(as>0));
    
    if verbosity,
    fprintf('Is the feasibilization problem solvable: %12.10f\n',...
	      norm((Aqwe*pinv(Aqwe)-eye(length(find(as>0))))*bqwe));
    end
    
    xqwe=zeros(n,1);
    xqwe(find(Aa_index>0)) = pinv(Aqwe)*bqwe;
    
    x = xqwe;
  else,
    feas_update = 1;
  end
  
  
  
  % Calculate graidient
  % Check for violated constraints
  f = A*x-b;

  f(Aa_index(find(Aa_index))) = 0;
  vc = find(f>1e-10);
  
  f(vc);
  
  g = zeros(n,1);
  for i=1:length(vc)
    g = g+A(vc(i),:)';
  end

  if verbosity,
      fprintf('\nNew Iteration\n');
      fprintf('Max constraint: %6.15f\n',max(f))
      fprintf('Slack variable: %6.15f\n',x(n))
      
      fv = A*x-b;
      fv = [NaN; fv];
      
      Aqwe=A(find(as>0),find(Aa_index>0));
      bqwe = b(find(as>0));
      
      cond(Aqwe)
      xqwe=zeros(n,1);
      xqwe(find(Aa_index>0)) = inv(Aqwe)*bqwe;
      
      fv2 = A*xqwe-b;
      fv2 = [NaN; fv2];
      
      [Aa_index fv(Aa_index+1) fv2(Aa_index+1)]
  
      %x=xqwe;
      
  end

  
  
  
  % Calculate Lagrange multipliers
  %St = -inv(Aa)';
  l = -S'*g;
  
  if verbosity,
    %fprintf('Norm difference: %16.15f\n',norm(inv(Aa)-S));
  end
  
  % Check if there is a Lagrange multiplier less than zero
  lq = -1e-10;
  q = 0;
  
  for i=1:length(l)
    if (l(i)<lq),
      lq = l(i);
      q = i;
    end
  end
  
  if verbosity,
    %fprintf('Primary Constraints in Active Set:\n');
    %[As_index(find(As_index>0)) find(As_index>0) f(find(As_index>0))]
    norm(S)
    norm(g)
    l
    if q > 0,
      fprintf('Smallest Lag. mult. has index: %3.0f and is %15.13f\n',q,lq);
    end
  end
  
  % If a pseudo constraint with positive lagrange multiplier is
  % removed, the we must seach in the opposite direction in order
  % to decreas the cost function
  sig = 1;
  
  % If no Lagrange multiplier, remove a pseudo constraint (with non
  % zeros Lagrange multiplier?) Remove the biggest one!
  if q == 0,
    sig = -1;
    lq = 1e-10;
    pse_constr_index = find(Aa_index==0);
    if ~isempty(pse_constr_index),
      for i=1:length(pse_constr_index),
	if l(pse_constr_index(i)) > lq, % Only positive lag mults...
	  q = pse_constr_index(i);
	  lq = l(pse_constr_index(i));
	end
      end
      if verbosity,
	fprintf('No negative lagrange multiplier, removing pseudo constraint with index %3.0f\n',q);
      end
    end
  end
  
  
  if q == 0,
    % Only primary constraints in the active set, or no nonzero
    % multiplier corresponding to a pseudo constraint no feasible
    % solution exists
    if verbosity,
      l
    end
    fprintf('No feasible solution exists\n')
    x = x(1:n-1);
    as = [];
    return
  end
      
  %s = sig*S(q,:)';
  s = -sig*S(:,q);
  
  
  ff = A*x-b;
  ss = A*s;
  
  check_index = ones(m,1);
  check_index = check_index - as;
  check_index(find(ss<=1e-10)) = 0;
  
  check_index = find(check_index);
  alpha = inf;
  alpha_index = 0;
  for i=1:length(check_index),
    j = check_index(i);
    qwe = -ff(j)/ss(j);
    if and(qwe > 0, qwe < alpha),
      alpha = qwe;
      alpha_index = j;
    end
  end
  
  if verbosity,
    %ff(alpha_index)
    %ss(alpha_index)
    if alpha < inf,
      fprintf('For alpha, constr %3.0f is restrictive, with  alpha = %15.13f\n',alpha_index,alpha);      
    end
  end
    
    
  % If alpha has been "small" for 3 rounds, make larger step...
  if alpha < 1e-3,
    nbr_small_alpha = nbr_small_alpha + 1;
    if nbr_small_alpha >=2,
      alpha = alpha + 1;
      nbr_small_alpha = 0;
    end
  else
    nbr_small_alpha = 0;
  end
    
  % If no constraint becomes violated for any alpha>0, find the
  % last constraint to become unviolated. In this case, update of
  % the active set is done.
  if alpha == inf, 
    
    alpha = 0;
    
    check_index = ones(m,1);
    check_index = check_index - as;
    check_index(find(ss>=0)) = 0;
        
    check_index = find(check_index);
    for i=1:length(check_index),
      j = check_index(i);
      qwe = -f(j)/ss(j);
      if and(qwe > 0, qwe >= alpha),
	alpha = qwe;
	alpha_index = j;
      end
    end
    
    if verbosity,
      fprintf('No restricting constraint! For alpha, constr %3.0f is is the last to become unviolated, with  alpha = %15.13f\n',alpha_index,alpha);      
       
    end

    alpha = alpha + 10; % Add some so that the constraint is not active.
    
    x = x + alpha*s;
    % In this case, no feasibilizing update should be done!
    feas_update = 0;
    
  else
 
    % Calculate cycling matrix
    cycling_detector = [[alpha_index; cycling_detector(1:4,1)]...
			cycling_detector(:,1:4)];
    
    % Detect cycles of period 2, 3 and 4
    if sum(cycling_detector(1:2,1)~=cycling_detector(1:2,3))==0,
      % Cycling detected!
      alpha = alpha + 1;
      feas_update = 0;
      if verbosity,
	fprintf('Cycling detected!\n');
      end
      elseif sum(cycling_detector(1:3,1)~=cycling_detector(1:3,4))==0
      % Cycling detected!
      alpha = alpha + 1;
      feas_update = 0;
      if verbosity,
	fprintf('Cycling detected!\n');
      end
    elseif sum(cycling_detector(1:4,1)~=cycling_detector(1:4,5))==0
      % Cycling detected!
      alpha = alpha + 1;
      feas_update = 0;
      if verbosity,
	fprintf('Cycling detected!\n');
      end
    else % No cycling, proceed as usual

      % Invert S
      p = alpha_index;
      p_old = p;
      q_old = q;
      ap_old = A(p,:);
      aq_old = Aa(q,:);
      
      eq=[zeros(q_old-1,1); 1; zeros(n-q_old,1)];
      vq=S(:,q_old);	
      d=(ap_old*S)';
      S=S-vq*(d-eq)'/d(q);
      
      
      Aa(q,:) = A(alpha_index,:);
      as(alpha_index) = 1;
      
      if Aa_index(q) > 0,
	as(Aa_index(q)) = 0;
      end
      
      Aa_index(q) = alpha_index;    
      
    
    end
    
    x = x + alpha*s;
    
  end
  
  
  fea_test = A*x-b;
  %fea_test(Aa_index(find(Aa_index))) = 0;
  
  if max(fea_test) <= 1e-8,
    fea =1;
  end

  if verbosity,
    fprintf('Most violated constraint has value: %10.15f\n',max(fea_test));
    fprintf('and indices:\n');
    find(fea_test==max(fea_test))
    %iter
    Aa_index_history = [Aa_index_history Aa_index];
    %qwe = toc
    %time_elapsed = time_elapsed+qwe
    pause
  end
  
  iter = iter + 1;

end

x = x(1:n-1);
as = [];

if verbosity,
  %x
  %f = A(1:n-2,1:n-1)*x-b(1:n-2);
  %max(f)
end

