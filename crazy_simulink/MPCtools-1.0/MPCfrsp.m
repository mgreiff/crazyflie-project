function [Sys_CL, Sys_S, Sys_CS, Sys_SU, F, H, K, h] = MPCfrsp(md,fignbr)
%
% [Sys_CL, Sys_S, Sys_CS, Sys_SU, F, H, K, h] = MPCfrsp(md)
%
% calculates the frequency responses of the MPC controller. When no 
% constraints active, the MPC controller is a linear
% controller which may be analyzed using standard methods. MPCfrsp
% calculates the following functions:
%
%    Sys_CL    Closed loop system: from r to z
%    Sys_S     Sensitivity function: from v to y
%    Sys_CS    Complimentary sensitivity function: from n to y
%    Sys_SU    Control signal sensitivity function: from n to u
%    F         See block diagram
%    H         See block diagram
%    K         See block diagram
%
% Sys_CL, Sys_S, Sys_CS and SYS_SU are also plotted. The functions
% are calculated using the block diagram
%
%
%                                            v
%                                           |
%                                           |
%           -----        -----     -----    v
%    r --->|  F  |-->+--|  K  |-->|  P  |-->+--> y 
%           -----    ^   -----     -----    |
%  	             |-      -----          |
%                    -------|  H  |---------+<-- n
%	                    ------
%	
% where v is a disturbance acting on the output and n is measurement
% noise. A vector of handles to the plots is given by h. If two
% arguments are given, plots are drawn in figure fignbr and fignbr+1.
%
% See also MPCInit
% 

del = ss(0,1,1,0,md.h);
int = ss(tf([1 0],[1 -1],md.h));

n = md.n;
P = ss(md.Ade(1:n,1:n),md.Bde(1:n,:),md.Cyde(:,1:n),zeros(md.pye, ...
						  md.me),md.h);



% Calculate Ks
Ksr = md.Ks(:,1:md.pze);
Ksx = md.Ks(:,md.pze+1:md.pze+md.ne);
Ksu = md.Ks(:,md.pze+md.ne+1:end);

% We must handle each case separately

if md.cmode == 0,

  % Calculate Fr
  Fr = Ksr;
  
  % Calculate Hr
  Hr = -Ksx;
    
  % Calculate Kr
  Kr = inv(minreal(eye(md.me) - int*Ksu*del ))*int;
  Kr = minreal(Kr);
  
  Mz = md.Czde(1:md.pz,1:md.n);
  Mdz = md.Dzde;
  M = eye(md.n);
  
elseif md.cmode == 1,
  Ksxi = Ksx(:,md.n+1:md.ne);
  Ksx = Ksx(:,1:n);
  
  % Calculate Fr
  Fr = Ksr(:,1:md.pz) + int*Ksxi;
  Fr = minreal(Fr);
  
  % Calculate Hr
  Hr = -Ksx + int*Ksxi*md.Czde(1:md.pz,1:md.n);
  Hr = minreal(Hr);
  
  % Calculate Kr
  Kr = inv(minreal(eye(md.me) - int*Ksu*del ))*int;
  Kr = minreal(Kr);
  
  Mz = md.Czde(1:md.pz,1:md.n);
  Mdz = md.Dzde(1:md.pz,1:md.m);
  M = eye(md.py);

elseif md.cmode == 2,
 
  % Calculate Fr
  Fr = Ksr;
  
  % Calculate Hr
  Hr = ss(md.Ade-md.K*md.Cyde,md.K,eye(md.ne),zeros(md.ne,md.pye), ...
	  md.h);
  Hr = -Ksx*Hr;
  Hr = minreal(Hr);
  
  % Calculate Kr
  Kr = ss(md.Ade-md.K*md.Cyde,md.Bde,eye(md.ne),zeros(md.ne,md.me), ...
	  md.h);
  
  Kr = inv(minreal(eye(md.me) - int*Ksu*del - int*Ksx*Kr))*int;
  Kr = minreal(Kr);

  Px = ss(md.Ade(1:n,1:n),md.Bde(1:n,:),eye(n),zeros(md.n, ...
						  md.me),md.h);
  

  
  
  Mz = md.Czde(:,1:n);
  Mdz= md.Dzde;
  M = eye(md.py);


  Sys_CL = Mz*inv(minreal(eye(md.n)+Px*Kr*Hr*md.Cyde(:,1:md.n)))*Px*Kr*Fr...
	   +Mdz*inv(minreal(eye(md.m)+Kr*Hr*P))*Kr*Fr;
  
  Sys_CL = minreal(Sys_CL);
  
elseif md.cmode == 3,
  
  Ksxi = Ksx(:,md.n+1:md.ne);
  Ksx = Ksx(:,1:n);
  
  % Calculate Fr
  Fr = Ksr(:,1:md.pz) + int*Ksxi;
  Fr = minreal(Fr);
  
  % Calculate Hr
  Hr = ss(md.Ade(1:n,1:n)-md.K*md.Cyde(:,1:n),md.K,eye(md.n),...
	  zeros(md.n,md.py),md.h);
  
  Hr = -Ksx*Hr + int*Ksxi*[eye(md.pz) zeros(md.pz,md.pye-md.pz)];
  Hr = minreal(Hr);
  
  % Calculate Kr
  Kr = ss(md.Ade(1:n,1:n)-md.K*md.Cyde(:,1:n),md.Bde(1:n,:),eye(md.n),...
	  zeros(md.n,md.me),md.h);
  Kr = inv(minreal(eye(md.me) - int*Ksu*del - int*Ksx*Kr))*int;
  Kr = minreal(Kr);
  
  Mz = [eye(md.pz) zeros(md.pz,md.pye-md.pz)];
  Mdz = md.Dzde(1:md.pz,1:md.m);
  M = eye(md.py);
  
elseif md.cmode == 4,
 
  % Calculate Fr
  Fr = Ksr;
  
  % Calculate Hr
  Hr = ss(md.Ade-md.K*md.Cyde,md.K,eye(md.ne),zeros(md.ne,md.pye), ...
	  md.h);
  Hr = -Ksx*Hr;
  Hr = minreal(Hr);
  
  % Calculate Kr
  Kr = ss(md.Ade-md.K*md.Cyde,md.Bde,eye(md.ne),zeros(md.ne,md.me), ...
	  md.h);
  
  Kr = inv(minreal(eye(md.me) - int*Ksu*del - int*Ksx*Kr))*int;
  Kr = minreal(Kr);

  Mz = [eye(md.pz) zeros(md.pz,md.pye-md.pz)];
  Mdz= md.Dzde;
  M = eye(md.py);
    
end

if (md.cmode~=2),
  Sys_CL = Mz*inv(minreal(eye(md.pye)+P*Kr*Hr))*P*Kr*Fr...
	   +Mdz*inv(minreal(eye(md.m)+Kr*Hr*P))*Kr*Fr;
  
  Sys_CL = minreal(Sys_CL);
end
  
% Get the Sensitivity function
Sys_S = minreal(M*inv(minreal(eye(md.pye)+P*Kr*Hr)));

% Get the complimentary sensitivity function
Sys_CS = M*inv(minreal(eye(md.pye)+P*Kr*Hr))*P*Kr*Hr;
Sys_CS = minreal(Sys_CS);

% Get noise sensitivity in control signal
Sys_SU = inv(minreal(eye(md.me)+Kr*Hr*P))*Kr*Hr;
Sys_SU = minreal(Sys_SU);
  

if (nargin>1),

  h(1)=figure(fignbr);
  clf
  subplot(2,2,1)
  sigma(Sys_CL)
  grid
  title('Closed Loop')
  subplot(2,2,2)
  sigma(Sys_S)
  grid
  title('Sensitivity Function')
  subplot(2,2,3)
  sigma(Sys_CS)
  grid
  title('Complimentary Sensitivity')
  subplot(2,2,4)
  sigma(Sys_SU)
  grid
  title('Control Sensitivity to Noise')

  h(2)=figure(fignbr+1);
  clf
  bode(Sys_CL)
  grid
end

F=Fr;
K=Kr;
H=Hr;