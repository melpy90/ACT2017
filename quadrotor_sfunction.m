function [sys,x0,str,ts] = quadrotor_sfunction(t,x,u,flag)



switch flag

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0
    [sys,x0,str,ts]=mdlInitializeSizes(init);

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1
    sys=mdlDerivatives(t,x,u);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3
    sys=mdlOutputs(t,x,u);

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
function [sys,x0,str,ts]=mdlInitializeSizes()

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
x0=init;
% end mdlInitializeSizes
%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,u)
gg = 9.81;
LL = 2;
bb = 1;
psi =0;
theta=0;
phi=0;
g=9.81;

p=[0 0 0]'; % position vector
v=[0 0 0]'; % linear velocity
w=[0 0 0]'; % angular velocity
gamma =[0 0 0]'; % torques vector
f = 100  % it is the sum of four motor trusts
m = 20; % quadrotor mass
e3= [0 0 1]';

% w x J*w = Omega

Omega = cross(w, J*w);

% standard quadrotor model
%p_dot= R*v;
%v_dot= -Omega*v +g*R'*e3-f*e3/m; % cos e omega?
%R_dot= R*Omega;
%J*w_dot = -Omega*J*w+gamma;


%rotation matrix
R=[cos(psi)*cos(theta) -sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi) sin(psi)*sin(phi)+cos(psi)*sin(theta)*cos(phi);
   sin(psi)*cos(theta) cos(psi)*cos(phi)+sin(psi)*sin(theta)*sin(phi) -sin(psi)*cos(phi)+sin(psi)*sin(theta)*cos(phi);
   -sin(theta) sin(phi)*cos(theta) cos(theta)*cos(phi)

]
% inertial matrix

J=diag([jx jy jz]);


phi_dot= w(1,1) + w(2,1)*sin(phi)*tan(theta)+ w(3,1)*cos(phi)*tan(theta);
theta_dot= w(2,1)*cos(phi)-w(3,1)*sin(phi);
psi_dot = w(2,1)*sin(phi)*(1/cos(theta))+ w(3,1)*cos(phi)*(1/cos(theta));

v1_dot = w(3,1)*v(2,1)- w(2,1)*v(3,1)-g*sin(theta);
v2_dot = -w(3,1)*v(1,1) + w(1,1)*v(3,1)+g*sin(phi)*cos(theta);
v3_dot = w(2,1)*v(1,1)- w(1,1)*v(2,1)+g*cos(theta)*cos(phi)-f/m;


p_dot= w(2,1)*w(3,1)*((jy-jz)/jx) + gamma(1,1)/jx;
q_dot= w(1,1)*w(3,1)*((jz-jx)/jy) + gamma(2,1)/jy;
r_dot= w(1,1)*w(2,1)*((jx-jy)/jz) + gamma(3,1)/jz;

sys = [phi_dot;theta_dot;psi_dot;v1_dot;v2_dot;v3_dot;p_dot;q_dot;r_dot];

% end mdlDerivatives
%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u)

%sys=[];
sys = x;

% end mdlOutputs
