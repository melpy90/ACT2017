close all; 
clear all; 
clc;

jx = 21
jy = 23
jz = 34
%f = [0 0 0]'
f0 = [200 20 30 20]'; % 4 motor thrusts
t = 0
% p=[0 0 0]'
MAXITER = 10
p = zeros(3,MAXITER);
% Sampling the position of the quadrotor for integer time for 1 to MAXITER
for t=1:1:MAXITER % t in seconds
[sys,x0,str,ts] = mdlInitializeSizes(1);
[sys pos f0] = mdlDerivatives(t, x0, 13,jx,jy,jz,f0);
%[sys pos f] = mdlDerivatives(t, x0, 13,jx,jy,jz,f,f0);
% sys=mdlOutputs(1, x0, 13); 
p(:,t) = pos;
end
mdlOutputs(p);