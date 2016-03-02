function [xopt, lambda, J, x_hist] = qp_as(H,f,A,b,x0)

% 
% [xopt, lambda, J, x_hist] = qp_as(H,f,A,b,x0)
%
% solves the quadratic programming problem
%
%     min 0.5*x'*H*x + f'*x
%       st
%     A*x <= b
% 
% using an active set method. An initial solution is supplied in x0.
%
% The return arguments are:
%
%    xopt    The optimal solution
%    lambda  The lagrange multipliers
%    J       The optimal value of the cost function
%    x_hist  History of x during the optimization run
%
% The algorithm is based on Fletcher (1987) p. 240.
%
% See also: getfeasible
%

% Contains check for cycling...

Ph = H;
ph = f;
Om = A;
om = b;

verbosity = 0;

th0 = x0;
J = [];

x_hist = th0;

if verbosity,
  qwe_qr = 0;
  qwe_alpha =0;
  qwe_inf = 0;
  qwecpu = cputime;
  global a_set_his;
  a_set_his = [];
end

iter = 0;

[m,n] = size(Om);

if verbosity,
  Omega_red = Om;
  omega_red = om;
  save slask2.mat Omega_red omega_red
end

if isempty(Om),
  xopt = -inv(Ph)*ph;
  lambda = [];
  return  
end


n = length(th0);
nc = length(om);

if verbosity,
  fmax = max((Om*th0-om)')

  if fmax > -1e-5,
    fprintf('Initial solution infeasible, max feasibility: %15.14g\n', ...
	    fmax);
  else
    fprintf('Initial solution feasible, max feasibility: %15.14g\n', ...
	    fmax);
  end
end

% Check feasibility
if max((Om*th0-om)')>-1e-5,
  
  if verbosity, qwe_feas=cputime; end
  [th0,a_set]=getfeasible(Om,om);
  
  if max(max((Om*th0-om)')) > 0,
    fprintf('No feasible starting point found, max violated constraint: %12.10f\n',max(max((Om*th0-om)')));
    lambda = zeros(m,1);
    J = 1/2*th0'*Ph*th0+ ph'*th0;
    xopt = th0;
    return
  end
  
  
  if verbosity, 
    qwe_feas=cputime-qwe_feas;
    fprintf('Maximum feasibility after getfeasible: %12.10f\n\n', ...
	    max((Om*th0-om)'));
    pause;
  end
else
  a_set = [];  
end

if verbosity,
  V = 1/2*th0'*Ph*th0 + ph'*th0;
  V_old = V;
end

finish = 0;

th = th0;
lam = zeros(nc,1);
a_dec = zeros(nc,1);


% The cholesky factorisation of Ph could be made in advance
L = chol(Ph)';
Linv = inv(L);


% Variable to check for cycling
nbr_no_progress = 0;
while finish~=1, 

  %a_set
  %disp('*****')
  % Build active set constraints
 
  Oma = Om(a_set,:);
  oma = om(a_set,:);
  
  if verbosity, 
    fprintf('New iteration\n');
    fprintf('Size of active set: %4.0f\n',length(a_set))
    fprintf('Rank of active set Omega matrix: %4.0f\n',rank(Oma))
    fprintf('Condition number of Omega: %4.5f\n',cond(Oma))
  end

  n_as = length(oma);
  
  %Find direction, delta theta
  if n_as > 0,
    LinvOma = Linv*Oma';
  else
    LinvOma = Linv;
  end

  %size(LinvOma)
  %rank(LinvOma)
  %svd(LinvOma)
  
  if verbosity, qwe2 = cputime; end
  
  [Q, R] = qr(LinvOma);
  Rran = rank(R);
  R = R(1:Rran,:);
  Q1 = Q(:,1:Rran);
  
  if verbosity,
    fprintf('Size of R matrix: %3.0f x % 3.0f\n',size(R));
    fprintf('Rank of R matrix: %3.0f\n',rank(R));
  end
    
  if n_as == 0,
    H = Linv'*Linv;
  else
    H = Linv'*(eye(n)-Q1*Q1')*Linv;
  end

  dth = -H*(ph+Ph*th);
  
  th_temp = th + dth;

  th_test = th-H*(Ph*th+ph);
  
  % Feasibilization step...
  %ph_tr = ph+Ph*th;
  %th_fea = quadprog(Ph,ph_tr,[],[],Oma,zeros(size(oma)));
    
  %th_temp = th_fea;
  %dth = th_temp-th;
  
  %dth = th_fea;
  %th_temp = th + dth;
  
  a_set_old = a_set;
  
  feasib = Om*th_temp-om;
  
  % Only check the constraints that are not in the active set
  a_set_compl = 1:length(om);
  a_set_compl(a_set) = 0;
  a_set_compl=a_set_compl(find(a_set_compl>0));
  %feasib(a_set_compl);
  %max(feasib(a_set_compl'))

  if verbosity, 
    fprintf('Cost function reduction potential: %12.10f\n',...
	    1/2*th_temp'*Ph*th_temp + th_temp'*ph-V); 
    fprintf('Is the initial solution feasible? Max feasibility: %12.15f\n',max(feasib')); 
  end

  % Feasible solution? Only check constraint not in the active
  % set...
  if verbosity,
    fprintf('Cputime for QR up to feasib test: %6.5f\n',cputime-qwe2);
    qwe_qr = qwe_qr+cputime-qwe2;
  end
  
  if max(feasib(a_set_compl)')>1e-6,
    if verbosity,
      qwe2 = cputime; 
      disp('Unfeasible sol');
    end
    progress = 1;
    
    O = om-Om*th;
    o = Om*dth;

    %[O(88) o(88)]
    
    checkindex = 1:nc;
    checkindex(a_set) = 0;
    checkind = find(o<1e-10);
    checkindex(checkind) = 0;
    [bogus1, bogus2, checkindex] = find(checkindex);
    
    
    %o(checkindex);
    alpha_dec = O(checkindex)./o(checkindex);
    alpha_min = min(alpha_dec);
    alpha_index = find(alpha_dec == alpha_min);
    
    if verbosity,
      alpha_dec_tmp = sortrows([alpha_dec checkindex']);
      fprintf('The first and last entries of minimal alphas:\n');
      disp(alpha_dec_tmp(1:min(3,length(alpha_dec_tmp)),:))
      %disp(alpha_dec_tmp(end-10:end,:))
    end
    
    alpha = alpha_dec(alpha_index(1));
    ind = checkindex(alpha_index)';
       
    %if and(alpha == 0,~isempty(find(a_set==ind))),
    if alpha == 0,
      progress = 0;
    else
      a_set = [a_set; ind(1)];
      a_set = sort(a_set);
    end

    if verbosity, 
      fprintf('alpha: %6.15f, index: ',alpha);
      for i=1:length(ind),
	fprintf('%4.0f, ',ind(i));
      end
      fprintf('\n');
    end
    
    th = th + alpha*dth;
    
    if verbosity,
      fea = Om*th-om;
      fprintf('New feasibility: %12.10f\n',max(fea));
      fprintf('Max feasibility for index:\n')
      disp(find(fea==max(fea)))
      fprintf('Cputime for calculation of alpha: %6.5f\n',cputime- ...
	      qwe2)
      qwe_alpha = qwe_alpha+cputime-qwe2;
    end
    
  else
    progress = 0;
  end
  
  if progress == 0;

    if verbosity, 
      qwe2 = cputime; 
      disp('Feasible sol or no progress')
    end
    
    T = -Linv'*Q1*inv(R)';
    U = -inv(R)*inv(R)';
    
    if max(feasib') < 1e-6,
      nbr_no_progress = 0;
      
      if verbosity, disp('Feasible sol'); end
      th = th_temp;
      if length(a_set)>0,
	%disp('lambda')
	lam_tq = T'*ph+U*oma;
      else
	lam_tq = [];
      end
      
      if length(lam_tq)>0,
	lam_t = zeros(nc,1);
	lam_t(a_set_old) = lam_tq;
	lam = lam_t;
      else
	lam = zeros(nc,1);
      end
    
      % Remove constraints ? 
      ind = find(lam<0);
      if length(ind) > 0,
	min_ind = find(min(lam)==lam);
	min_ind = min_ind(1);
	
	a_set_tmp = [];
	for k = 1:length(a_set)
	  if a_set(k) ~= min_ind;
	    a_set_tmp = [a_set_tmp; a_set(k)];
	  end  	
	end
	a_set_tmp;
	lam(min_ind) = 0; 
	a_set = a_set_tmp;
      end
    else
      % Check if there has been no progress for the last two
      % iterations. In that case, remove one constraint and continue.
      nbr_no_progress = nbr_no_progress + 1;
      if nbr_no_progress >2,
	nbr_no_progress = 0;
	if isempty(a_set),
	  a_set = ind;
	else,
	  %a_set = a_set(1:end-1);
	  constr_to_remove = round(rand(1)*length(a_set)+0.5);
	  a_set = [a_set(1:constr_to_remove-1);...
	  	   a_set(constr_to_remove+1:end)];
	  end
      end      
    end
    
    a_set;

    if verbosity,
      fprintf('Weighted norm of termination vector: %15.12f\n',norm(Ph*th+ ...
						  Om'*lam+ph)/sqrt(n))
      fprintf('Minimum lagrange multiplier: %15.12f\n',min(lam))
    end
    
    if and(norm(Ph*th+Om'*lam+ph)/sqrt(n)<5e-3,...
	   min(lam')>=0)
      finish = 1;
    end
    
    if verbosity,
      feas = Om*th-om;
      fprintf('Maximum constraint %15.12f\n',max(feas));
    end;
    
    if verbosity,
      fprintf('Cputime for No progress section: %6.5f\n',cputime- ...
	      qwe2)
      qwe_inf = qwe_inf+cputime-qwe2;
    end
    
  end

  if verbosity,
    %qwe1 = 1:length(om);
    %qwe2 = zeros(1,length(om));
    %qwe2(a_set) = qwe1(a_set)';
    %a_set_his = [a_set_his; ... 
% 		 qwe2];
    a_set
  end
  
  if verbosity,
    V_old = V;
    V = 1/2*th'*Ph*th+ ph'*th;
    V-V_old;
    fprintf('Decrement of loss function: %15.14f\n',V-V_old);
    fprintf('Iteration finished\n\n');
    format long
    %th
    pause
  end

  J = [J;1/2*th'*Ph*th+ ph'*th];
  x_hist = [x_hist th];
  
  
  %pause
  iter = iter+1;
  
  if iter>1000, break, end
     
end
xopt = th;
lambda = lam;

if verbosity,
  %fprintf('Total time for feasibility: %6.5f\n',qwe_feas)
  fprintf('Total time for QR section: %6.5f\n',qwe_qr)
  fprintf('Total time for alpha section: %6.5f\n',qwe_alpha)
  fprintf('Total time for no progress section: %6.5f\n',qwe_inf)
  fprintf('Total time: %6.5f\n',cputime-qwecpu)
end

