%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
% function sys=mdlOutputs(t,x,u)
function mdlOutputs(sys)
%sys=[];
plot3(sys(1,:),sys(2,:),sys(3,:), 'color',[ColorMatrix(1,:),ColorMatrix(2,:),ColorMatrix(3,:)]);
hold on;
grid on;