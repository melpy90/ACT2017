%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================

function [sys pos f] = mdlDerivatives(t,x,u,jx,jy,jz,f)
g = 9.81;
LL = 0.2; % lenght between two motors
% bb = 1;

             % the angles: pitch roll and yawl are
%beeing calculated as written by trigonometrics at t=0

%psi  
psi = acos(LL/sqrt((f(3,1)-f(3,2))^2 +LL^2));
%theta;
theta = acos(LL/sqrt((f(3,1)-f(3,3))^2 +LL^2));
%phi;
phi= acos(LL/sqrt((f(3,1)-f(3,4))^2 +LL^2));



% p=[0 0 0]'; % initial position vector
v=[sum(f(3,:))*cos(psi) sum(f(3,:))*cos(theta) sum(f(3,:))*cos(phi)]'; % linear velocity       
% w=[diff(psi,t), diff(theta,t), diff(phi,t)]'; % angular velocity  d(angle)/dt


MAXITER = 10
for t=1:MAXITER
    
    % computation of angles at t=1 an so on for obtain angular velocity
    % with angle variation in dt.
    
psi_1 = acos(LL/sqrt((f(3,1)-f(3,2))^2 +LL^2));

theta_1 = acos(LL/sqrt((f(3,1)-f(3,3))^2 +LL^2));

phi_1= acos(LL/sqrt((f(3,1)-f(3,4))^2 +LL^2));

w = [(psi_1-psi)/t (theta_1-theta)/t (phi_1-phi)/t]';
end

gamma =[0.2 0.1 0.1]'; % torques vector

m = 0.25; % quadrotor mass
e3= [0 0 1]';

% w x J*w = Omega

% inertial matrix

%J=diag([jx jy jz]);

% Omega = cross(w, J*w);

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


D = diag((1/(2*p))*(v).^2*Cd*A); % drag coefficient


%% DYNAMICS
phi_dot= w(1,1) + w(2,1)*sin(phi)*tan(theta)+ w(3,1)*cos(phi)*tan(theta);
theta_dot= w(2,1)*cos(phi)-w(3,1)*sin(phi);
psi_dot = w(2,1)*sin(phi)*(1/cos(theta))+ w(3,1)*cos(phi)*(1/cos(theta));

v1_dot = w(3,1)*v(2,1)- w(2,1)*v(3,1)-g*sin(theta);
v2_dot = -w(3,1)*v(1,1) + w(1,1)*v(3,1)+g*sin(phi)*cos(theta);
v3_dot = w(2,1)*v(1,1)- w(1,1)*v(2,1)+g*cos(theta)*cos(phi)-sum(f(3,:))/m;


p_dot= w(2,1)*w(3,1)*((jy-jz)/jx) + gamma(1,1)/jx;
q_dot= w(1,1)*w(3,1)*((jz-jx)/jy) + gamma(2,1)/jy;
r_dot= w(1,1)*w(2,1)*((jx-jy)/jz) + gamma(3,1)/jz;

% Component by component
for i=1:4
    f(:,i) = f(:,i)-D*ev+m*g+R*f(:,i); % f with aerodynamics drag
    
    
end


sys = [phi_dot theta_dot psi_dot;v1_dot v2_dot v3_dot;p_dot q_dot r_dot];
pos = t*[v1_dot v2_dot v3_dot]';
clc;

% end mdlDerivatives