%% MOPPC parameters
MOPPCparam.h = 0.002;
MOPPCparam.degR = 1;
MOPPCparam.degT = 1;
MOPPCparam.degS = 1;

% Observer pole
MOPPCparam.ao = -0.99;

% Known process model
MOPPCparam.b0 = 0.379932887351898;
MOPPCparam.b1 = 0.373321059237816;
MOPPCparam.a1 = -1.947644233748935;
MOPPCparam.a2 = 0.948696191846759;

% Reference model
MOPPCparam.bm0 = 0.1;

% Test with poles on point or slightly off
MOPPCparam.am1 = -1.943454396449151;
MOPPCparam.am2 = 0.945009782207162;
MOPPCparam.am1 = -1.947644233748935;
MOPPCparam.am2 = 0.948696191846759;