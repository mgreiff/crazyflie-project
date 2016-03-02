function [x, u, y, z, zPredTraj, uPredTraj] = MPCSim(md,r,d)
%
% [x, u, y, z, zPredTraj, uPredTraj] = MPCSim(md,r,d)
% 
% simulates the MPC controller specified by the data object md. The
% reference trajectory for the controlled output is given by r and a
% load disturbance acting on the input is given by d.
%
% The returned arguments are:
%
%    x          State trajectories of the system
%    u          Control signals
%    y          Measured Outputs
%    z          Controlled Outputs
%    zPredTraj  The predicted controlled outputs at each sample
%    uPredTraj  The predicted control inputs at each sample
%
% See also MPCInit, MPCOptimizeSol, MPCfrsp
%

s=r;

[qwen,qwem] = size(s);
if qwem~=md.pz,
  error('The reference value matrix must have the same number of columns as the number of rows of Czd');
end

[qwen3,qwen4] = size(d);
if qwen~=qwen3,
   error('The matrices s and d must have the same number of rows');
end

[qwen,qwem] = size(d);
if qwem~=md.m,
  error('The input disturbance matrix must have the same number of columns as the number of columns of Bd');
end

global x_est;
% Initialize
%md = MPCInit(Ad,Bd,Cyd,Czd,Dzd,Ccd,Dcd,Hp,Hw,zblk,Hu,ublk, ...
%	    du_max,du_min,u_max,u_min,z_max, ...
%	    z_min,Q,R,W,V,h,cmode);

t=0;
cont= 1;

x_plant = zeros(md.n,1);
x_hat = zeros(md.ne,1);

u_old = zeros(md.me,1);

x_est = [];
y_old = zeros(md.pye,1);
u = [];
x = [];
y = [];
z = [];
duOpt = zeros(md.me*md.Hp,1);

zPredTraj = [];
uPredTraj = [];

x_hat1=zeros(3,1);
x_hat2=zeros(3,1);

oldsp = inf*ones(md.pz,1);

while cont == 1

  %*************** "Measurement"  
  % Get outputs
  y_plant = md.Cyde(:,1:md.n)*x_plant;
  %***************  
  
  % Get observer values
  if md.cmode == 0, % State feedback
  
    x_hat(1:md.n) = x_plant;
    refval = s(t+md.Hw+1,:);
    
  elseif md.cmode == 1; % State feedback + explicit integrators

    % State feedback
    x_hat(1:md.n) = x_plant;
    
    % Take care of integrator states
    x_hat(md.n+1:md.n+md.nbr_int) = x_hat(md.n+1:md.n+md.nbr_int) + ... 
                               s(t+md.Hw+1,1:md.nbr_int)' - ...
		               md.Czde(1:md.nbr_int,1:md.n)*x_hat(1:md.n);
			       
    refval = [s(t+md.Hw+1,:) zeros(1,md.nbr_int)];
    
  elseif md.cmode == 2; % Observer
    
    % Update Observer
    x_hat = md.Ade*x_hat + md.Bde*u_old + md.K*(y_old - md.Cyde* ...
						x_hat);
    
    refval = s(t+md.Hw+1,:);
  
  elseif md.cmode == 3; % Observer + explicit integrators
  
    % Notice that it is assumed that the first m entries of y is
    % actually the controlled outputs, and integral action is
    % applied to these.
    
    % Update observer
    x_hat(1:md.n) = md.Ade(1:md.n,1:md.n)*x_hat(1:md.n) + ...
	            md.Bde(1:md.n,:)*u_old +...
	            md.K*(y_old - md.Cyde(:,1:md.n)*x_hat(1:md.n));
    
    % Take care of integrator states
    x_hat(md.n+1:md.n+md.nbr_int) = x_hat(md.n+1:md.n+md.nbr_int) + ... 
                               s(t+md.Hw+1,1:md.nbr_int)' ...
	                       - y_plant(1:md.nbr_int); 
    
    % Set reference value
    refval = [s(t+md.Hw+1,:) zeros(1,md.nbr_int)];
    
  elseif md.cmode == 4; % Disturbance observer
    
    x_hat = md.Ade*x_hat + md.Bde*u_old + md.K*(y_old - md.Cyde* ...
						x_hat);
       
    refval = s(t+md.Hw+1,:);
    
       
  end

  y_old = y_plant;
  
  % Optimize
  [duOpt,zOpt] = MPCOptimizeSol(x_hat,u_old,duOpt,refval,md);
  %disp('Optimization finished');

  
  % Store predicted values
  zPredTraj = [zPredTraj reshape(zOpt,md.pze,md.Hp)'];
  uPT = reshape(duOpt,md.me,md.Hu)';
  uPT(1,:) = u_old' + duOpt(1:md.me)';
  uPT = cumsum(uPT);
  uPredTraj = [uPredTraj uPT];
  
  % Calculate control signal to store

  u_old = u_old + duOpt(1:md.me);
  u = [u; u_old'];
  x_est = [x_est; x_hat';];
  x = [x; x_plant'];
  z = [z; (md.Czde(:,1:md.n)*x_plant)'];
  y = [y; y_plant';];
  t = t + 1;

  if rem(t,50)==0,
    fprintf('.\n')
  else
    fprintf('.')
  end
  
  
  
  if t >= length(s)-md.Hp-1
    cont = 0;
  end
  
  %************Update Plant*****
  x_plant = md.Ade(1:md.n,1:md.n)*x_plant +...
	    md.Bde(1:md.n,:)*(u_old+d(t,:)');
  %*****************************
  
end
fprintf('\n')
%warning backtrace;