function md = MPCInit(Ad,Bd,Cyd,Czd,Dzd,Ccd,Dcd,Hp,Hw,zblk,Hu,ublk, ...
		      du_max,du_min,u_max,u_min,z_max, ...
		      z_min,Q,R,W,V,h,cmode,solver)
%
% md = MPCInit(Ad,Bd,Cyd,Czd,Dzd,Ccd,Dcd,Hp,Hw,zblk,Hu,ublk, ...
%		      du_max,du_min,u_max,u_min,z_max, ...
%		      z_min,Q,R,W,V,h,cmode,solver)
% 
% creates the data structure used by the MPC controller. A discrete
% time model, with sampling interval h, of the controlled system is
% assumed, 
%
%    x(k+1) = Ad*x(k) + Bd*u(k)
%    y(k)   = Cyd*x(k)
%    z(k)   = Czd*x(k) + Dzd*u(k)
%    z_c(k) = Ccd*x(k) + Dcd*u(k)
%
% where z(k) are the controlled outputs, y(k) the measured
% outputs and z_c(k) the constrained outputs. The constraints
% of the system are given by
%
%    du_min <= u(k)-u(k-1) <= du_max, k in Ip
%    u_min  <=     u(k)    <= u_max, k in Ip
%    z_min  <=    z_c(k)   <= z_max, k in Iu
%
% Ip and Iu are the sets of samples for which the controlled and
% control variables are included in the cost function and for which
% the constraints are enforced. The first sample to be included in the
% optimization procedure is Hw, and the total number of samples in the
% prediction horizon is Hp. The entries in the array zblk indicates
% the time distance between two consecutive predicted samples. Also,
% the last entry in zblk is used as distance for the remaining samples
% (if length(zblk) < Hp).
%
%   Example:
%      Assume Hw = 1, Hp = 6 and zblk = [1 2 2 5]. 
%      Then the samples [1 2 4 6 11 16] will be included.
% 
% Equivalently, Hu indicates the number of predicted control moves 
% to be calculated at each sample. The array ublk contains the blocking
% factors indicating over which sampling intervals the control
% signal should be constant. For example, a blocking factor of 2
% indicates that 2 consecutive control signals are equal.
%
%    Example:
%       Assume that Hu = 3, ublk = [1 2].
%       Then it is assumed that at each sample, for the predicted
%       control signal, u_p, we have that u_p(k+1)=u_p(k+2) and
%       u_p(k+3)=u_p(k+4). Only up(k), u_p(k+1) and u_p(k+3) will be
%       determined. 
%
% Q and R are weighting matrixes for the cost function, where Q
% penalizes the controlled outputs and R penalizes control increments.
% W and V are weighting matrices for the design of the Kalman filter
% used to estimate the state and load disturbances. W is the
% covariance matrix of the state and V is the covariance matrix of the
% measurement noise.
%
% cmode specifies the controller mode. The following modes
% are supported:
%
% 0: State feedback. The arguments W and V are not used, and may be
%    set to []. Cyd is assumed to be identity.
%
% 1: State feedback and explicit integrators acting on the
%    controlled outputs, specified by Czd. The Q matrix should be
%    extended to include weights for the integrator states as well
%    as the controlled outputs. The arguments W and V are not used,
%    and may be set to [].
%
% 2: Observer based output feedback.
%
% 3: Observer based output feedback and explicit integrators acting
%    on the controlled outputs, specified by Czd. The Q matrix should be
%    extended to include weights for the integrator states as well
%    as the controlled outputs. The controlled variables should be
%    the same as the first pz (pz = dim(z)) measured variables.
%
% 4: Observer based output feedback with a disturbance model that
%    achieves error free tracking. Constant load disturbances
%    are assumed on the control input, and the system is augmented
%    to include also the disturbance states. If the number of
%    measured outputs exceeds the number of inputs, constant load
%    disturbances on the additional measured outputs are assumed. 
%    The controlled variables should be the same as the first pz
%    measured variables. Also, the number of controlled outputs
%    should be equal to the number of inputs.
%
% The argument solver should be a string containing one of the
% alternatives 'qp_as', 'qp_ip' or 'quadprog', indicating which
% solver should be used to solve the quadratic programming problem.
% Notice that 'quadprog' requires Optimization Toolbox.
%
% See also MPCOptimizeSol, MPCSim and MPCfrsp


global md;
md = struct('Ps',[],... % Psi matrix,
	    'Ga',[],... % Gamma matrix,
	    'Th',[],... % Theta matrix
	    'Ps_c',[],... % Psi matrix, constraints
	    'Ga_c',[],... % Gamma matrix, constraints
	    'Th_c',[],... % Theta matrix, constraints
	    'Qq',[],... % Controlled states weights
	    'Rr',[],... % Delta u weigths
	    'H',[],... % Hessian of QP problem
	    'F_con',[],... % Constraints 
	    'f_con',[],... 
	    'Gam_con',[],... % Constraints 
	    'gam_con',[],... 
	    'W_con',[],... % Constraints 
	    'w_con',[],... 
	    'K',[],... % Observer Gain
	    'Kmpc',[],... % Controller gain
	    'Ks',[],...
	    'Ade',[],... % A matrix for extended discretized system
	    'Bde',[],... % B matrix for extended discretized system
	    'Cyde',[],... % Cy matrix for extended discretized system
	    'Czde',[],... % Cz matrix for extended discretized system
	    'Dzde',[],... % Dz matrix for extended discretized system
	    'Ccde',[],... % Cc matrix for extended discretized system
	    'Dcde',[],... % Dc matrix for extended discretized system
	    'n',[],... % Number of states for system
	    'm',[],... % Number of inputs for system
	    'py',[],... % Number of measured outputs for system
	    'pz',[],... % Number of controlled outputs for the system
            'pc',[],... % Number of constrained outputs for the system
	    'ne',[],... % Number of states for extended system
	    'pze',[],... % Number of controlled outputs for extended system
	    'Hp',Hp,... % Prediction horizon
	    'Hu',Hu,... % Control signal horizon
	    'Hw',Hw,... % Prediction horizon start sample
	    'h',h,... % Sampling interval
            'cmode',cmode,... % Controller mode
	    'solver',solver,...
	    'nbr_int',[]); % The number of explicit integrators


% Check size of system matrices
[na,ma] = size(Ad);
[nb,mb] = size(Bd);
[nc,mc] = size(Cyd);

py = nc;

if or(na~=ma,or(nb~=na,mc~=na)),
  error('Size of matrices Ad, Bd, Cyd do not match')
end

% Check Cz
[pz, nz] = size(Czd);
if na~=nz,
    error('Czd must have the same number of columns as Ad');
end

% Check Dz
[qwen,qwem] = size(Dzd);
if or(qwen~=pz,qwem~=mb),
  error('Dzd must have the same number of rows as Czd and the same number of column as Bd')
end

% Check Cc
[pc, ncc] = size(Ccd);
if na~=ncc,
    error('Ccd must have the same number of columns as Ad');
end

% Check Dc
[qwen,qwem] = size(Dcd);
if or(qwen~=pc,qwem~=mb),
  error('Dcd must have the same number of rows as Ccd and the same number of column as Bd')
end

% Check Q
[qwen,qwem] = size(Q);
if or(cmode == 0,or(cmode == 2,cmode ==4))
  if or(qwen~=qwem,qwen~=pz)
    error('The Q matrix must be square and have the same number of rows as Czd');
  end
else % Explicit integrators case
  if or(qwen~=qwem,qwen~=(pz+pz))
    error('The Q matrix must be square and have twice as many rows as Czd to account for the integrator states.');
  end
end

% Check R
[qwen,qwem] = size(R);
if or(qwen~=qwem,qwem~=mb)
  error('The R matrix must be square and have the same number of columns as Bd');
end
  
% Check constraints
qwe1 = length(du_max);
qwe2 = length(du_min);
qwe3 = length(u_max);
qwe4 = length(u_min);
qwe5 = length(z_max);
qwe6 = length(z_min);

if or(qwe1~=mb,or(qwe2~=mb,or(qwe3~=mb,qwe4~=mb))),
  error('The length of the constraint vectors for the control signal must match the size of the Bd matrix');
end
[qwen,qwem] = size(Ccd);
if or(qwe5~=qwen,qwe6~=qwen),
  error('The length of the constraint vectors for the constrained outputs must match the size of the Czd matrix');
end

% Check W matrix
[qwen,qwem] = size(W);
if or(cmode==2,cmode==3),
  if or(qwen~=qwem,qwen~=na),
    error('The matrix W must be square and have the same number of rows as Ad')
  end
elseif (cmode==4),
  if or(qwen~=qwem,qwen~=(na+py)),
    error('The matrix W must be square and have dimension equal to the number of rows of Ad plus the number of row of Cy')
  end
end
  
% Check V matrix  
if or(cmode == 2,or(cmode ==3,cmode == 4)),
  [qwen,qwem] = size(V);
  if or(qwen~=qwem,qwen~=nc),
    error('The matrix V must be square and have the same number of rows as Cyd')
  end
end

% Check solver field
if or(strcmp(solver,'qp_as'),or(strcmp(solver,'qp_ip'), ...
				strcmp(solver,'quadprog')))==0,
  error('Invalid solver, use qp_as, qp_ip or quadprog.')

end


md.n = na;
md.m = mb;
md.py = py;
md.pz = pz;
md.pc = pc;

disp('Setting up matrices...')

if md.cmode == 0, % State feedback no observer, no pure integral action

  Cyd = eye(na);
  [py, qwe] = size(Cyd);
    
  md.py = py;

  md.Ade = Ad;
  md.Bde = Bd;
  md.Cyde = Cyd;
  md.Czde = Czd;
  md.Dzde = Dzd;
  md.Ccde = Ccd;
  md.Dcde = Dcd;
  
  [md.pze, md.ne] = size(md.Czde);
  [md.pye, md.ne] = size(md.Cyde);
  [md.ne, md.me] = size(md.Bde);  

  md.nbr_int = 0;

elseif md.cmode == 1, % State feedback and explicit integral action
     
  Cyd = eye(na);
  [py, n] = size(Cyd);
 
  md.py = py;
  
  nbr_int = pz;
  md.nbr_int = nbr_int;
 
  md.Ade = [Ad zeros(na,nbr_int);
	    -Czd eye(nbr_int)];
  md.Bde = [Bd; zeros(nbr_int,mb)];
  md.Cyde = [Cyd zeros(py,nbr_int)];
  md.Czde = [Czd zeros(pz,nbr_int);
	     zeros(nbr_int,n) eye(nbr_int)];
  md.Dzde = [Dzd;zeros(nbr_int,mb)];
  md.Ccde = [Ccd zeros(pc,nbr_int)];
  md.Dcde = Dcd;
  
  [md.pze, md.ne] = size(md.Czde);
  [md.pye, md.ne] = size(md.Cyde);
  [md.ne, md.me] = size(md.Bde);  
   
elseif md.cmode == 2, % Output feedback no disturbance observer no pure
		      % integral action
  md.Ade = Ad;
  md.Bde = Bd;
  md.Cyde = Cyd;
  md.Czde = Czd;
  md.Dzde = Dzd;
  md.Ccde = Ccd;
  md.Dcde = Dcd;
  
  [md.pze, md.ne] = size(md.Czde);
  [md.pye, md.ne] = size(md.Cyde);
  [md.ne, md.me] = size(md.Bde);
  
  % Build observer

  md.K = dlqr(md.Ade',md.Cyde',W,V,zeros(md.ne,md.pye))';

elseif md.cmode == 3, % Output feedback, no disturbance observer,
                      % explicit integral action
		      
  % Notice that it is assumed that the first pz entries of y is
  % actually the controlled outputs!		      
		      
  nbr_int = pz;
  md.nbr_int = nbr_int;
  
  % Check that Cyd has at least pz rows
  if py<pz,
    error('The number of measured outputs is less than the number of controlled outputs. All controlled outputs subject to integral action must be measured.')
  end
  
  % Make sure that the first pz rows of Czd equals the first m rows
  % of Cyd.
  if (Cyd(1:pz,:)~=Czd(1:pz,:))~=0,
    error('The first m rows of Czd must equal the first m rows of Cyd')
  end
      
  md.Ade = [Ad zeros(na,nbr_int);
	    -Czd eye(nbr_int)];
  md.Bde = [Bd; zeros(nbr_int,mb)];
  md.Cyde = [Cyd zeros(py,nbr_int)];
  md.Czde = [Czd zeros(pz,nbr_int);
	     zeros(nbr_int,na) eye(nbr_int)];
  md.Dzde = [Dzd; zeros(nbr_int,mb)];
  md.Ccde = [Ccd zeros(pc,nbr_int)];
  md.Dcde = Dcd;
  
  [md.pze, md.ne] = size(md.Czde);
  [md.pye, md.ne] = size(md.Cyde);
  [md.ne, md.me] = size(md.Bde);
  
  % Build observer
  md.K = dlqr(md.Ade(1:na,1:na)',md.Cyde(:,1:na)',W,V,zeros(na,py))';

elseif md.cmode == 4, % Output feedback, disturbance observer
  
  % We assume that the first m measured outputs are controlled
  
   % Make sure that the first m rows of Czd equals the first m rows
  % of Cyd.
  if norm(Cyd(1:mb,:)-Czd(1:mb,:))~=0,
    error('The first m rows of Czd must equal the first m rows of Cyd')
  end
  
   % Set up extended system for use with observer. We assume constant
  % load disturbances on the inputs.
  md.Ade = [Ad Bd; zeros(mb,na) eye(mb)];
  md.Bde = [Bd; zeros(mb,mb)];
  md.Cyde = [Cyd zeros(py,mb)]; 
  md.Czde = [Czd zeros(pz,mb)]; 
  md.Dzde = Dzd;
  md.Ccde = [Ccd zeros(pc,mb)]; 
  md.Dcde = Dcd;
  
  % Also, assume constant output load disturbances on additional
  % outputs
  if py > mb,
    md.Ade = blkdiag(md.Ade,eye(py-mb));
    md.Bde = [md.Bde; zeros(py-mb,mb)];
    md.Cyde = [Cyd [zeros(mb,py);[zeros(py-mb,mb) eye(py-mb)]]];
    md.Czde = [md.Czde zeros(pz,py-mb)];
    md.Ccde = [md.Ccde zeros(pc,py-mb)];
  end 

  [md.pze, md.ne] = size(md.Czde);
  [md.pye, md.ne] = size(md.Cyde);
  [md.ne, md.me] = size(md.Bde);
  
  % Build observer
  md.K = dlqr(md.Ade',md.Cyde',W,V,zeros(md.ne,md.pye))';
  
end

% Build precomputable matrices
T = [];
Tc = [];
Ps = [];
Ga = [];
Th = [];
Qq = Q;
Rr = [];

disp('Preparing matrix setup...')
% Constraint matrices etc...
z_block = [];
gz_block = zeros(0,1);
for k=1:md.pc
  qwe = [];
  qwe2 = [];
  if z_max(k) ~= inf,
    qwe = 1;
    qwe2 = z_max(k);
  end
  if z_min(k) ~= -inf,
    qwe = [qwe; -1];
    qwe2 = [qwe2; -z_min(k)];
  end
  if isempty(qwe),
    [qwen,qwem] = size(z_block);
    z_block = [z_block zeros(qwen,1)];
  else
    z_block = blkdiag(z_block,qwe);
  end
  gz_block = [gz_block; qwe2];
end

uf_block = [];
uw_block = [];
bu_block = zeros(0,1);
au_block = zeros(0,1);
for k=1:md.me
  qwef = [];
  qwew = [];
  qwe2 = [];
  
  if u_max(k) ~= inf,
    qwef = 1;
    qwe2 = u_max(k);
  end
  if u_min(k) ~= -inf,
    qwef = [qwef; -1];
    qwe2 = [qwe2; -u_min(k)];
  end 
  if isempty(qwef),
    [qwen,qwem] =size(uf_block);
    uf_block = [uf_block zeros(qwen,1)];
  else
    uf_block = blkdiag(uf_block,qwef);
  end
  bu_block = [bu_block; qwe2];
  
  qwe2 = [];
  if du_max(k) ~= inf,
    qwew = 1;
    qwe2 = du_max(k);
  end
  if du_min(k) ~= -inf,
    qwew = [qwew; -1];
    qwe2 = [qwe2; -du_min(k)];
  end 
  if isempty(qwew),
    [qwen,qwem] =size(uw_block);
    uw_block = [uw_block zeros(qwen,1)];
  else
    uw_block = blkdiag(uw_block,qwew);
  end
  au_block = [au_block; qwe2];
end

zSampInclude = []; % z samples to include (expressed as index)
zblklast = [];
if length(zblk) >= md.Hp-1,
  zblklast = md.Hp-1;
else
  zblklast = length(zblk);
end
if md.Hp > 1,
  zSampInclude = cumsum([md.Hw+1 zblk(1:zblklast) ...
		    zblk(zblklast)*ones(1,Hp-length(zblk)-1)]);
end
zSampInclude;
size(zSampInclude);

duSampInclude = []; % u samples to include (expressed as index)
ublklast = [];
if length(ublk) >= md.Hu-1,
  ublklast = md.Hu-1;
else
  ublklast = length(ublk);
end
if md.Hu > 1,
  duSampInclude = cumsum([1 ublk(1:ublklast) ...
		    ublk(ublklast)*ones(1,Hu-length(ublk)-1)]);
end
duSampInclude;
size(duSampInclude);

gz_block;
z_block;

[nbr_constr_ublk,qwe] = size(uf_block);  
[nbr_constr_dublk,qwe] = size(uw_block);  
F_con = [uf_block zeros(nbr_constr_ublk,md.me*duSampInclude(end)-md.me)] ;
f_con = bu_block;
Gam_con = z_block; % Accounts for constraints on Hw = 0;
gam_con = gz_block;
W_con = [uw_block zeros(nbr_constr_dublk,md.me*duSampInclude(end)-md.me)] ;
w_con = au_block;

disp('Building prediction matrices...')
% Create matrices from i = 1 to i = last to include
for k=1:zSampInclude(end),
  T = blkdiag(T,md.Czde);
  Tc = blkdiag(Tc,md.Ccde);
  Ps = [Ps; md.Ade^k];
  Qq = blkdiag(Qq,Q);
  if k == 1 
    Ga = [md.Bde];
    Th = [md.Bde zeros(md.ne,md.me*(duSampInclude(end)-1))];
  else
    Ga = [Ga; md.Ade^(k-1)*md.Bde + Ga((1+md.ne*(k-2)):(md.ne*(k-1)),:)];
    Th = [Th; Ga(1+md.ne*(k-1):(md.ne*k),:) ... 
	  Th(1+md.ne*(k-2):(md.ne*(k-1)),1:md.me*(duSampInclude(end)-1))];
  end
  Gam_con = blkdiag(Gam_con,z_block);
  gam_con = [gam_con; gz_block];
end
%disp('hej');
%size(Gam_con);
%size(gam_con);


for k=1:duSampInclude(end),
  Rr = blkdiag(Rr,R);
  
  if k < duSampInclude(end)
    F_con = [F_con; [zeros(nbr_constr_ublk,k*md.me)...
		     uf_block zeros(nbr_constr_ublk,md.me*...
				    duSampInclude(end)-(k+1)*md.me)]];
    f_con = [f_con; bu_block];
        
    W_con = [W_con; [zeros(nbr_constr_dublk,k*md.me)...
		     uw_block zeros(nbr_constr_dublk,md.me*...
				    duSampInclude(end)-(k+1)*md.me)]];
    w_con = [w_con; au_block];
  end       
end

for i=duSampInclude(end)-1:-1:1,
  F_con(:,(i-1)*md.me+1:i*md.me) = F_con(:,(i-1)*md.me+1:i*md.me)...
      +F_con(:,i*md.me+1:(i+1)*md.me);
end

% Create prediction matrices for constraints
Ps_c = Tc*Ps;
Ga_c = Tc*Ga;
Th_c = Tc*Th;

% Save original prediction matrices to calculate 
% terminal constraint.
Ps_terminal = Ps;
Ga_terminal = Ga;
Th_terminal = Th;

% Create prediction matrices for cost function
Ps = T*Ps;
Ga = T*Ga;
Th = T*Th;

% Take direct term into account for z
Ga_add = [];
Th_add = [];
Ga_add_c = [];
Th_add_c =[];
for k = 0:zSampInclude(end),
  if (k < duSampInclude(end)),
    Th_add = [Th_add; repmat(md.Dzde,1,k+1)...
	      zeros(md.pze,md.me*(duSampInclude(end)-k-1))];
    Th_add_c = [Th_add_c; repmat(Dcd,1,k+1)...
	      zeros(md.pc,md.me*(duSampInclude(end)-k-1))];
  else
    Th_add = [Th_add; repmat(md.Dzde,1,duSampInclude(end))];
    Th_add_c = [Th_add_c; repmat(Dcd,1,duSampInclude(end))];
  end 
  Ga_add = [Ga_add; md.Dzde];
  Ga_add_c = [Ga_add_c; Dcd];
  
end

% Add row(s) to take i = 0 into account
Ps = [md.Czde; Ps];
Th = [zeros(md.pze,md.me*duSampInclude(end)); Th];
Th = Th+Th_add;
Ga = [zeros(md.pze,md.me) ;Ga]+Ga_add;

Ps_c = [md.Ccde; Ps_c];
Th_c = [zeros(md.pc,md.me*duSampInclude(end)); Th_c];
Th_c = Th_c+Th_add_c;
Ga_c = [zeros(md.pc,md.me) ;Ga_c]+Ga_add_c;

% Matrix creation complete! 

disp('Post-creation matrix operations...')
% Take only specified samples into account
% Take blocking factors into account. Start with z
% Create vector with specified samples. First row corresponds to 
% k = 0. First sample to include is Hw
zRowInclude = (md.pze*ones(md.pze,1)*zSampInclude) ...
    -[md.pze-1:-1:0]'*ones(1,length(zSampInclude));
zRowInclude = reshape(zRowInclude,md.pze*(md.Hp),1);

size(Ps);
md.Ps = Ps(zRowInclude,:);
md.Ga = Ga(zRowInclude,:);
md.Th = Th(zRowInclude,:);
md.Qq = Qq(zRowInclude,zRowInclude);

zRowInclude_c = (md.pc*ones(md.pc,1)*zSampInclude) ...
    -[md.pc-1:-1:0]'*ones(1,length(zSampInclude));
zRowInclude_c = reshape(zRowInclude_c,md.pc*(md.Hp),1);

md.Ps_c = Ps_c(zRowInclude_c,:);
md.Ga_c = Ga_c(zRowInclude_c,:);
md.Th_c = Th_c(zRowInclude_c,:);

[qwen,qwem] = size(Gam_con);
nbr_constr_z = qwen/(zSampInclude(end)+1);
gamRowInclude = nbr_constr_z*ones(nbr_constr_z,1)*...
    zSampInclude -[nbr_constr_z-1:-1:0]'*ones(1,length(zSampInclude));
gamRowInclude = reshape(gamRowInclude,nbr_constr_z*md.Hp,1);

md.Gam_con = Gam_con(gamRowInclude,zRowInclude_c);
md.gam_con = gam_con(gamRowInclude);

% Now take the delta u:s into account
% Create vector with specified samples 
duColInclude = md.me*ones(md.me,1)*duSampInclude ...
    -[md.me-1:-1:0]'*ones(1,length(duSampInclude));
duColInclude = reshape(duColInclude,md.me*md.Hu,1);

md.Th = md.Th(:,duColInclude);
md.Th_c = md.Th_c(:,duColInclude);

[qwen,qwem] = size(F_con);
nbr_constr_uf = qwen/duSampInclude(end);
fRowInclude = nbr_constr_uf*ones(nbr_constr_uf,1)*...
    duSampInclude - [nbr_constr_uf-1:-1:0]'*ones(1, ...
						length(duSampInclude));
fRowInclude = reshape(fRowInclude,nbr_constr_uf*md.Hu,1);

[qwen,qwem] = size(W_con);
nbr_constr_uw = qwen/duSampInclude(end);
wRowInclude = nbr_constr_uw*ones(nbr_constr_uw,1)*...
    duSampInclude - [nbr_constr_uw-1:-1:0]'*ones(1, ...
						length(duSampInclude));
wRowInclude = reshape(wRowInclude,nbr_constr_uw*md.Hu,1);

md.Rr = Rr(duColInclude,duColInclude);

md.F_con=F_con(fRowInclude,duColInclude);
md.f_con=f_con(fRowInclude,:);
md.W_con=W_con(wRowInclude,duColInclude);
md.w_con=w_con(wRowInclude,:);

md.H = md.Th'*md.Qq*md.Th + md.Rr;

% Calculate gain matrix for the MPC controller
md.Kmpc = 1/2*inv(md.Th'*md.Qq*md.Th + md.Rr)*(2*md.Th'*md.Qq);
md.Kmpc = md.Kmpc(1:md.me,:);

md.Ks = md.Kmpc*[repmat(eye(md.pze),md.Hp,1) -md.Ps -md.Ga];

%******************* Include terminal constraint on z

term_block = [];
for i=1:md.pz,
  term_block = blkdiag(term_block,[1 -1]');
end

size(term_block);
size(zeros(2*md.pz,md.pz*(md.Hp-1)));

md.Gam_con_term = [zeros(2*md.pz,md.pz*(md.Hp-1)) term_block];

md.F_con_term = [md.F_con; md.F_con(end-2*md.m+1:end,:)];

%*******************************************************

