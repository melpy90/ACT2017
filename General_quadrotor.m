close all; clear all; clc;

psi = 0;
theta = 0;
phi = 0;
g = 9.81;

p =[0 0 0]'; % position vector
v =[0 0 0]'; % linear velocity
w =[0 0 0]'; % angular velocity
gamma =[0 0 0]'; % torques vector
f = 100  % it is the sum of four motor trusts
m = 20; % quadrotor mass
e3= [0 0 1]';

Omega = [0 0 0;
         0 0 0;
         0 0 0]

% initial state

x=[p(1,1) p(2,1) p(3,1) psi theta psi v(1,1) v(2,1) v(3,1) w(1,1) w(2,1) w(3,1)]';

% input 

u=[f gamma(1,1) gamma(2,1) gamma(3,1)]';

%output

y=[p(1,1) p(2,1) p(3,1)]'



%rotation matrix
R=[cos(psi)*cos(theta) -sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi) sin(psi)*sin(phi)+cos(psi)*sin(theta)*cos(phi);
   sin(psi)*cos(theta) cos(psi)*cos(phi)+sin(psi)*sin(theta)*sin(phi) -sin(psi)*cos(phi)+sin(psi)*sin(theta)*cos(phi);
   -sin(theta) sin(phi)*cos(theta) cos(theta)*cos(phi)

]
% inertial matrix

J=[ jx 0 0;
    0 jy 0;
    0 0 jz]
%************************************************************************
% standard quadrotor model


p_dot= R*v                          % vel
v_dot = -Omega*v +g*R'*e3-(f*e3)*m; % acc ....... omega?
R_dot= R*Omega;
J*w_dot = -Omega*J*w+gamma;


%*************************************************************************
% let omega, belongs to so(3), be the screw-symmetric matric
% associated with w, and such that for beta (belongs to the real^3),
% omegabeta= w X beta


%parametrization equation

phi_dot= w(1,1) + w(2,1)*sin(phi)*tan(theta)+ w(3,1)*cos(phi)*tan(theta);
theta_dot= w(2,1)*cos(phi)-w(3,1)*sin(phi);
psi_dot = w(2,1)*sin(phi)*(1/cos(theta))+ w(3,1)*cos(phi)*(1/cos(theta))

v1_dot = w(3,1)*v(2,1)- w(2,1)*v(3,1)-g*sin(theta);
v2_dot = -w(3,1)*v(1,1) + w(1,1)*v(3,1)+g*sin(phi)*cos(theta);
v3_dot = w(2,1)*v(1,1)- w(1,1)*v(2,1)+g*cos(theta)*cos(phi)-f/m;


p_dot= w(2,1)*w(3,1)*((jy-jz)/jx) + gamma(1,1)/jx;
q_dot= w(1,1)*w(3,1)*((jz-jx)/jy) + gamma(2,1)/jy;
r_dot= w(1,1)*w(2,1)*((jx-jy)/jz) + gamma(3,1)/jz;





model(f, gamma)





