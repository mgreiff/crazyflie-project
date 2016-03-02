function [sys,x0,str,ts] = MPCController(t,x,u,flag,md)
%
%   [sys,x0,str,ts] = MPCController(t,x,u,flag,md)
%
% is an S-function implementing the MPC controller intended for use
% with Simulink. The argument md, which is the only user supplied
% argument, contains the data structures needed by the controller. The
% input to the S-function block is a vector signal consisting of the
% measured outputs and the reference values for the controlled
% outputs. The output of the S-function block is a vector signal
% consisting of the control variables and the estimated state vector,
% potentially including estimated disturbance states.

switch flag,
 case 0
  [sys,x0,str,ts] = mdlInitializeSizes(md); % Initialization
 case 2
  sys = mdlUpdates(t,x,u,md); % Update discrete states
  
 case 3
  sys = mdlOutputs(t,x,u,md); % Calculate outputs
  
 case {1, 4, 9} % Unused flags
  sys = [];
  
 otherwise
  error(['unhandled flag = ',num2str(flag)]); % Error handling
end
% End of dsfunc.

%==============================================================
% Initialization
%==============================================================

function [sys,x0,str,ts] = mdlInitializeSizes(md)

% Call simsizes for a sizes structure, fill it in, and convert it 
% to a sizes array.

sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = md.ne + md.pye + md.me + md.Hu*md.me;
sizes.NumOutputs     = md.me+md.ne;
sizes.NumInputs      = md.pye+md.pz;
sizes.DirFeedthrough = 1; % Matrix D is non-empty.
sizes.NumSampleTimes = 1;
sys = simsizes(sizes); 
x0 = zeros(md.ne + md.pye + md.me + md.Hu*md.me,1);   

global x;
x = zeros(md.ne + md.pye + md.me + md.Hu*md.me,1);   
% Initialize the discrete states.
str = [];             % Set str to an empty matrix.
ts  = [md.h 0];       % sample time: [period, offset]
% End of mdlInitializeSizes
		      
%==============================================================
% Update the discrete states
%==============================================================
function sys = mdlUpdates(t,x,u,md)

sys =  x;
% End of mdlUpdate.

%==============================================================
% Calculate outputs
%==============================================================
function sys = mdlOutputs(t,x_i,u,md)

global x;

fprintf('Update start, t=%4.2f\n',t)

x_hat = x(1:md.ne,:);
y_old = x(md.ne+1:md.ne+md.pye,:);
u_old = x(md.ne+md.pye+1:md.ne+md.pye+md.me,:);
duOpt = x(md.ne+md.pye+md.me+1:md.ne+md.pye+md.me+md.me*md.Hu,:);
y_plant = u(1:md.pye,:);
r = u(md.pye+1:md.pye+md.pz);

% Get observer values
if md.cmode == 0,
  
  x_hat(1:md.n) = y_plant;
  refval = r;
  
elseif md.cmode == 1;
  
  % State feedback
  x_hat(1:md.n) = y_plant;
  
  % Take care of integrator states  
  x_hat(md.n+1:md.n+md.nbr_int) = x_hat(md.n+1:md.n+md.nbr_int) + ... 
      r(1:md.nbr_int) - md.Czde(1:md.nbr_int,1:md.n)*x_hat(1:md.n);
  
  refval = [r' zeros(1,md.nbr_int)]';
  
elseif md.cmode == 2;
  
  % Update Observer
  x_hat = md.Ade*x_hat + md.Bde*u_old + md.K*(y_old - md.Cyde* ...
					      x_hat);
  
  y_old = y_plant;
  
  refval = r;
  
elseif md.cmode == 3;
  
  % Notice that it is assumed that the first pz entriew of y is
  % actually the controlled outputs!
  
  % Take care of integrator states
  x_hat(md.n+1:md.n+md.nbr_int) = x_hat(md.n+1:md.n+md.nbr_int) + ... 
      r(1:md.nbr_int) - y_plant(1:md.nbr_int);
  
  % Update observer
  x_hat(1:md.n) = md.Ade(1:md.n,1:md.n)*x_hat(1:md.n) + ...
      md.Bde(1:md.n,:)*u_old +...
      md.K*(y_old - md.Cyde(:,1:md.n)*x_hat(1:md.n));
  
  y_old = y_plant;
  
  % Set reference value
  refval = [r' zeros(1,md.nbr_int)]';
  
elseif md.cmode == 4;
  
  % Update observer
  x_hat = md.Ade*x_hat + md.Bde*u_old + md.K*(y_old - md.Cyde* ...
					      x_hat);

  y_old = y_plant;
  
  %set reference value
  refval = r;
  
end

% Optimize
%[duOpt,zOpt] = MPCOptimizeSolSC(x_hat,u_old,duOpt,refval,md);
[duOpt,zOpt] = MPCOptimizeSol(x_hat,u_old,duOpt,refval,md);

% Calculate control signal to store
u_old = u_old + duOpt(1:md.me);

x(1:md.ne,1) = x_hat;
x(md.ne+1:md.ne+md.pye,:) = y_old;
x(md.ne+md.pye+1:md.ne+md.pye+md.me,:) = u_old;
x(md.ne+md.pye+md.me+1:md.ne+md.pye+md.me+md.me*md.Hu,:) = duOpt;


sys = [x(md.ne+md.pye+1:md.ne+md.pye+md.me,:);
       x(1:md.ne,1);];

% End of mdlOutputs.