%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
% function sys=mdlOutputs(t,x,u)
function mdlOutputs(sys)
%sys=[];
sys
plot3(sys(1,:),sys(2,:),sys(3,:));
xlabel('value of x')
ylabel('value of y')
zlabel('value of z')
title('Quadrotor Trajectory')
text(0,1, 'Trajectory')


hold on;
grid on;