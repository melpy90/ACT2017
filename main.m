close all; 
clear all; 
clc;

%
% Coefficients of diagonal matrix J (inertial matrix) J=diag([jx jy jz]);
jx = 2.1
jy = 2.3
jz = 3.4
f0 = zeros(3,4);

% set each motor force
temp = [50 30 10 30]; 

for i=1:4
    f0(3,i) = temp(i); % 4 motor thrusts
end
t = 0
% p=[0 0 0]'
MAXITER = 10  % time sampling...seconds set the same in mdlDerivatives

f=f0; % for first time
temp = zeros(3,MAXITER);

% Sampling the position of the quadrotor for integer time for 1 to MAXITER
for t=1:1:MAXITER % t are integer seconds
    
[sys,x0,str,ts] = mdlInitializeSizes(1);

% [sys pos f0] = mdlDerivatives(t, x0, 13,jx,jy,jz,f0);
[sys pos f] = mdlDerivatives(t, x0, 13,jx,jy,jz,f);
%sys=mdlOutputs(1, x0, 13); 
temp(:,t) = pos;
end

mdlOutputs(temp);