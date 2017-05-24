% Just as a prototype of this project (unused in final version of this
% model)

function [sys,x0,str,ts] = quadrotor_sfunction(t,x,u,init,flag,jx,jy,jz,x0)



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
    [sys,x0,str,ts] = mdlDerivatives(t,x,u,jx,jy,jz);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3
    [sys,x0,str,ts]=mdlOutputs(t,x,u);

  %%%%%%%%%%%%%%%%%%%
  % Unhandled flags %
  %%%%%%%%%%%%%%%%%%%
  case { 2, 4, 9 },
    [sys,x0,str,ts] = [];

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end
% end csfunc


