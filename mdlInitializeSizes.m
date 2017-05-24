%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts]=mdlInitializeSizes(init)

sizes = simsizes;
sizes.NumContStates  = 12; % states
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 12; % output
sizes.NumInputs      = 4; % input
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);

str = [];
ts  = [0 0];
x0 = init;
% end mdlInitializeSizes