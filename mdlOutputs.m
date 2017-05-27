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
hold on;
grid on;