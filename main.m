close all; 
% clear all; 
clc;

jx = 50
jy = 40
jz = 20
f = 100
t = 0
% p=[0 0 0]'
MAXITER = 30
p = zeros(3,MAXITER);
% Sampling the position of the quadrotor for integer time for 1 to MAXITER
for t=1:1:MAXITER
[sys,x0,str,ts] = mdlInitializeSizes(1);
[sys pos] = mdlDerivatives(t, x0, 13,jx,jy,jz,f,p);
% sys=mdlOutputs(1, x0, 13); 
p(:,t) = pos;
end
mdlOutputs(p);