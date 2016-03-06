function [duPred, zPred, J] = MPCOptimizeSol(x_est,u_last,du_old,r,md)
%
% [duPred, zPred, J] = MPCOptimizeSol(x_est,u_last,du_old,r,md) 
%
% solves the MPC optimization problem. duPred and zPred are the
% predicted control inputs and controlled outputs for the specified
% prediction horizons.
%
% An estimate of the current state vector is given by x_est, and
% u_last is the last applied control input. As an initial starting
% point for the optimization algorithm, the last predicted control
% input vector, du_old, is supplied. r is the desired set point for
% the controlled outputs and md is the data structure containing
% matrices needed to solve the optimization problem.
%
% See also MPCInit, MPCSim and MPCfrsp
%


% Build reference trajectory
r = repmat(r,1,md.Hp);
r = reshape(r,1,md.pze*(md.Hp));
%r = r((md.Hw)*md.pze+1:(md.Hp)*md.pze);

T_run = r';

size(T_run);
size(md.Ps*x_est);
size(md.Ga*u_last);

E_run = T_run - md.Ps*x_est - md.Ga*u_last;

G_run = 2*md.Th'*md.Qq*E_run;

% Set up constraint matrices
Omega=[md.F_con; md.Gam_con*md.Th_c; md.W_con];

omega=[-md.F_con(:,1:md.me)*u_last + md.f_con;
       -md.Gam_con*(md.Ps_c*x_est+md.Ga_c*u_last) + md.gam_con;
       md.w_con];

% Solve optimization problem
du0 = [du_old(md.me+1:md.me*md.Hu); du_old(md.Hu*md.me-md.me+1:md.me*md.Hu)];

qweH = md.H;
%save slask qweH G_run Omega omega du0
warning off

if strcmp(md.solver,'qp_as')
  
  [duPred, l, J, duhist] = qp_as(2*md.H,-G_run,Omega,omega,du0);
    
elseif strcmp(md.solver,'qp_ip')
  
  [duPred, l, J, duhist] = qp_ip(2*md.H,-G_run,Omega,omega,du0);

else
  
  [duPred, J] = quadprog(2*md.H,-G_run,Omega,omega);
  
end
    

J = J+E_run'*md.Qq*E_run;

warning backtrace;

zPred = md.Ps*x_est + md.Ga*u_last + md.Th*duPred;

zp = reshape(zPred,md.pze,md.Hp)';
%figure(10)
%clf
%stairs(zp)
%grid

up = reshape(duPred,md.me,md.Hu);
up = cumsum([u_last up]')';
up = up(:,2:end)';
%figure(11)
%clf
%stairs(up)
%grid

 