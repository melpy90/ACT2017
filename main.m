close all; 
clear all; 
clc;

jx = 21
jy = 23
jz = 34
f0 = zeros(3,4);
temp = [200 199 198 197];
for i=1:4
    f0(3,i) = temp(i); % 4 motor thrusts
end
t = 0
% p=[0 0 0]'
MAXITER = 100

f=f0; % for first time
temp = zeros(3,MAXITER);
% Sampling the position of the quadrotor for integer time for 1 to MAXITER
for t=1:1:MAXITER % t in seconds
    
[sys,x0,str,ts] = mdlInitializeSizes(1);
% [sys pos f0] = mdlDerivatives_simone(t, x0, 13,jx,jy,jz,f0);
[sys pos f] = mdlDerivatives(t, x0, 13,jx,jy,jz,f);
% sys=mdlOutputs(1, x0, 13); 
temp(:,t) = pos;
end
mdlOutputs(temp);