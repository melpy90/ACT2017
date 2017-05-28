%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function [sys pos f0] = mdlDerivatives(t,x,u,jx,jy,jz,f0)
gg = [0 0 -9.81]';
 LL = 0.2; % lenght between two motors
% bb = 1;




%psi =deg2rad(90);              % vanno calcolate in base le 4 F...

psi = arccos( LL/sqrt((f0(1,1)-f0(2,1))^2 +LL^2));
%theta=deg2rad(60);
theta= arccos( LL/sqrt((f0(1,1)-f0(3,1))^2 +LL^2));
%phi=deg2rad(30);
phi= arccos( LL/sqrt((f0(1,1)-f0(4,1))^2 +LL^2));
g=9.81;

% p=[0 0 0]'; % initial position vector
v=[sum(f0)*cos(psi) sum(f0)*cos(theta) sum(f0)*cos(phi)]'; % linear velocity        % vanno calcolate in base le 4 F...
w=[diff(psi,t) diff(theta,t) diff(phi,t)]'; % angular velocity  d(angle)/dt
gamma =[0 0 0]'; % torques vector
% f = 100  % it is the sum of four motor trusts
m = 20; % quadrotor mass
e3= [0 0 1]';

% w x J*w = Omega

% inertial matrix

J=diag([jx jy jz]);

Omega = cross(w, J*w);

% standard quadrotor model
%p_dot= R*v;
%v_dot= -Omega*v +g*R'*e3-f*e3/m; % cos e omega?
%R_dot= R*Omega;
%J*w_dot = -Omega*J*w+gamma;


%Drag force applied to each rotor...


%sum(f0); % total thrust 
p=1.225;  %air density kg/m^3
A= 0.025; % area section of quadrotor.
Cd=0.09 % drag coefficient depending of the quadrotor shape.



%rotation matrix
R=[cos(psi)*cos(theta) -sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi) sin(psi)*sin(phi)+cos(psi)*sin(theta)*cos(phi);
   sin(psi)*cos(theta) cos(psi)*cos(phi)+sin(psi)*sin(theta)*sin(phi) -sin(psi)*cos(phi)+sin(psi)*sin(theta)*cos(phi);
   -sin(theta) sin(phi)*cos(theta) cos(theta)*cos(phi)

]

ev= [1/v(1,1) 1/v(2,1) 1/v(3,1)]'; % versore rispetto la direzione di velocit?
ed= [0 0 -1]'; %  versore in direzione z-


D = (1/(2*p))*(v).^2*Cd*A; % drag coefficient



f = -D*ev+m*gg+R*sum(f0)*e3; % f with aerodynamics drag

phi_dot= w(1,1) + w(2,1)*sin(phi)*tan(theta)+ w(3,1)*cos(phi)*tan(theta);
theta_dot= w(2,1)*cos(phi)-w(3,1)*sin(phi);
psi_dot = w(2,1)*sin(phi)*(1/cos(theta))+ w(3,1)*cos(phi)*(1/cos(theta));

v1_dot = w(3,1)*v(2,1)- w(2,1)*v(3,1)-g*sin(theta);
v2_dot = -w(3,1)*v(1,1) + w(1,1)*v(3,1)+g*sin(phi)*cos(theta);
v3_dot = w(2,1)*v(1,1)- w(1,1)*v(2,1)+g*cos(theta)*cos(phi)-f/m;


p_dot= w(2,1)*w(3,1)*((jy-jz)/jx) + gamma(1,1)/jx;
q_dot= w(1,1)*w(3,1)*((jz-jx)/jy) + gamma(2,1)/jy;
r_dot= w(1,1)*w(2,1)*((jx-jy)/jz) + gamma(3,1)/jz;




sys = [phi_dot theta_dot psi_dot;v1_dot v2_dot v3_dot;p_dot q_dot r_dot];
pos = t*[v1_dot v2_dot v3_dot]';
clc;

% end mdlDerivatives