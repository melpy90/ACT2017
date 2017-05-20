function [sys,x0,str,ts] = quadrotor_Sfunction(t,x,u1,u2,flag)
%CSFUNC An example MATLAB file S-function for defining a continuous system.  
%   Example MATLAB file S-function implementing continuous equations: 
%      x' = Ax + Bu
%      y  = Cx + Du
%   
%   See sfuntmpl.m for a general S-function template.
%
%   See also SFUNTMPL.
  
%   Copyright 1990-2009 The MathWorks, Inc.




switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes(t,x,u1,u2);

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u1,u2);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,xd,yd,td);

  %%%%%%%%%%%%%%%%%%%
  % Unhandled flags %
  %%%%%%%%%%%%%%%%%%%
  case { 2, 4, 9 },
    sys = [];

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end
% end csfunc

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts]=mdlInitializeSizes(x,u1,u2)

sizes = simsizes;
sizes.NumContStates  = 12;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 4;
sizes.NumInputs      = 4;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);
x0  = zeros(2,1);
str = [];
ts  = [0 0];

% end mdlInitializeSizes
%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,u1,u2)

global gg
global mm

g= 9.81; 
m= 0.8;
Ix= 2;
x_des = 1.5;

global xeq
global u1eq
global u2eq
global Ix;

u1 = F + m*g;
if (F==0) 
    u1= u1eq;
    
end

u2 = Ix*x_des;


if (u2==0)
    u2 = u2eq;
end




y_des = (u1/m)*sin(x) ;
z_sys = -g + (u1/m)*(cos(x)) ;
x_des = u2/Ix;




% end mdlDerivatives
%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u1,u2)

sys = y_des + z_des + x_des;

% end mdlOutputs
