function [xopt, lambda, J, x_hist] = qp_ip(H,f,A,b,x0)
% 
% [xopt, lambda, J, x_hist] = qp_ip(H,f,A,b,x0)
%
% solves the quadratic programming problem
%
%     min 0.5*x'*H*x + f'*x
%       st
%     A*x <= b
% 
% using a primal-dual interior point method. An initial solution is
% supplied in x0, but is not used efficiently by the algorithm in the
% current implementation. 
%
% The return arguments are:
%
%    xopt    The optimal solution
%    lambda  The lagrange multipliers
%    J       The optimal value of the cost function
%    x_hist  History of x during the optimization run
%
% The algorithm is based on Wright (1997).
%
% See also: qp_as
%

Q=H;
c=f;
C=-A;
d=-b;


[m,n] = size(C);

tol = 1e-5;

verbosity = 0;

x_hist = x0;

zeta = 100000;
x = x0; % Optimization variables
z = zeta*ones(m,1); % Dual variables
s = zeta*ones(m,1); % Slack varables

not_optimal = 1;

tau = 3;

term_norm = norm([Q c; C d],inf);

iter = 0;

J = [];

D = [];

while not_optimal, 
  
  if verbosity,
    %tic
    fprintf('\nNew iteration***\n')
  end
  
  mu = z'*s/m;
  Jac_red = Q + C'*(repmat(z./s,1,n).*C);
  
  z_over_s = z./s;
  
  F_red = -Jac_red*x - c +C'*z + C'*(d.*z_over_s);

  dx_aff = Jac_red\F_red;

  dz_aff = z_over_s.*(-C*dx_aff-C*x+d);
  
  ds_aff = -s + C*dx_aff + C*x - d;
  
  % find largest possible 0 < alpa <= 1
  % is the step feasible?
  zs = [z; s];
  dzs_aff = [dz_aff;ds_aff];
  if min(zs+dzs_aff)>0,
    alpha_aff = 1;
  else
  
    check_index = find(dzs_aff<0); % check only negative values,
                                   % the others are ok
    all_alpha = -zs(check_index)./dzs_aff(check_index);
    
    alpha_aff = min(all_alpha);
    
  end

  alpha_aff;
  
  mu_aff = (z+alpha_aff*dz_aff)'*(s+alpha_aff*ds_aff)/m;
  
  sigma = (mu_aff/mu)^tau;

  F_red = F_red + C'*((sigma*mu -dz_aff.*ds_aff)./s) ;

  dx = Jac_red\F_red;
  
  dz = z_over_s.*(-C*dx-C*x+d) + (1./s).*(sigma*mu - dz_aff.* ...
       ds_aff);
  
  ds = -s + C*dx + C*x - d;
  
  % find largest possible 0 < alpa <= 1
  % is the step feasible?
  zs = [z; s];
  dzs = [dz;ds];
  if min(zs+dzs)>0,
    alpha = 1;
  else
  
    check_index = find(dzs<0); % check only negative values,
                                   % the others are ok
    all_alpha = -zs(check_index)./dzs(check_index);
    
    alpha = 0.999*min(all_alpha);
    
  end
  
  x = x + alpha*dx;
  z = z + alpha*dz;
  s = s + alpha*ds;
  
  if and(mu <= tol, (norm([Q*x+c-C'*z;C*x-s-d],inf) + s'*z)/ ...
	 term_norm<=tol),  
    not_optimal = 0;
  end

  iter = iter + 1;
  
  J = [J; 1/2*x'*Q*x+c'*x;];
  D = [D; z'*s/m];

  x_hist = [x_hist x];
  
  if verbosity,
    fprintf('Iteration time %5.4f\n',0) %toc)
    fprintf('cost function = %18.15f\n',x'*Q*x+c'*x);
    fprintf('alpha_aff = %15.13f\n',alpha_aff);
    fprintf('alpha = %15.13f\n',alpha);
    fprintf('The (normalized) norm of the residuals is %18.15f\n', ...
	    (norm([Q*x+c-C'*z;C*x-s-d],inf) + s'*z)/term_norm) 
    
    
    fprintf('The duality gap is %18.15f\n',s'*z/m);
    %pause
  end
    
end

iter;
xopt = x;
lambda = z;

