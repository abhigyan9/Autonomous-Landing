function [sys,x0,str,ts,simStateCompliance] = uav_dynamics(t,x,u,flag,uav_P)

switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(uav_P);

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u,uav_P);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2,
    sys=mdlUpdate(t,x,u);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u);

  %%%%%%%%%%%%%%%%%%%%%%%
  % GetTimeOfNextVarHit %
  %%%%%%%%%%%%%%%%%%%%%%%
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9,
    sys=mdlTerminate(t,x,u);

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

% end sfuntmpl

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(uav_P)

%
% call simsizes for a sizes structure, fill it in and convert it to a
% sizes array.
%
% Note that in this example, the values are hard coded.  This is not a
% recommended practice as the characteristics of the block are typically
% defined by the S-function parameters.
%
sizes = simsizes;

sizes.NumContStates  = 12;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 12;
sizes.NumInputs      = 6;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

%
% initialize the initial conditions
%
x0  = [...
    uav_P.pn0;...
    uav_P.pe0;...
    uav_P.pd0;...
    uav_P.u0;...
    uav_P.v0;...
    uav_P.w0;...
    uav_P.phi0;...
    uav_P.theta0;...
    uav_P.psi0;...
    uav_P.p0;...
    uav_P.q0;...
    uav_P.r0;...
    ];

%
% str is always an empty matrix
%
str = [];

%
% initialize the array of sample times
%
ts  = [0 0];

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'DisallowSimState' < Error out when saving or restoring the model sim state
simStateCompliance = 'UnknownSimState';

% end mdlInitializeSizes

%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,uu, uav_P)

    pn    = x(1);
    pe    = x(2);
    pd    = x(3);
    u     = x(4);
    v     = x(5);
    w     = x(6);
    phi   = x(7);
    theta = x(8);
    psi   = x(9);
    p     = x(10);
    q     = x(11);
    r     = x(12);
    fx          = uu(1);
    fy          = uu(2);
    fz          = uu(3);
    tao_phi     = uu(4);
    tao_theta   = uu(5);
    tao_psi     = uu(6);
    
    %
    Jx = uav_P.Jx;
    Jy = uav_P.Jy;
    Jz = uav_P.Jz;
    grav = uav_P.gravity;

    pndot = cos(theta)*cos(psi)*u... 
            + (sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi))*v... 
            + (cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi))*w;
    
    pedot = cos(theta)*sin(psi)*u...
            + (sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi))*v...
            + (cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi))*w;
    
    pddot = -sin(theta)*u...
            + sin(phi)*cos(theta)*v...
            + cos(phi)*cos(theta)*w;
    
    %{
    udot = r*v - q*w - grav*sin(theta);
    vdot = p*w - r*u + grav*cos(theta)*sin(phi);
    wdot = q*u - p*v + grav*cos(theta)*cos(phi) - (1/uav_P.mass)*F;
    %}

    udot = r*v - q*w - grav*sin(theta) + fx/uav_P.mass;
    vdot = p*w - r*u + grav*cos(theta)*sin(phi) + fy/uav_P.mass;
    wdot = q*u - p*v + grav*cos(theta)*cos(phi) + fz/uav_P.mass;


    phidot = p + sin(phi)*tan(theta)*q + cos(phi)*tan(theta)*r;
    thetadot = cos(phi)*q - sin(phi)*r;
    psidot = (sin(phi)/cos(theta))*q + (cos(phi)/cos(theta))*r;

    pdot = (Jy-Jz)*q*r/Jx + tao_phi/Jx;
    qdot = (Jz-Jx)*p*r/Jy + tao_theta/Jy;
    rdot = (Jx-Jy)*p*q/Jz + tao_psi/Jz;

sys = [pndot; pedot; pddot; udot; vdot; wdot; phidot; thetadot; psidot; pdot; qdot; rdot];

% end mdlDerivatives

%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================
%
function sys=mdlUpdate(t,x,u)

sys = [];

% end mdlUpdate

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u)

sys = x;

% end mdlOutputs

%
%=============================================================================
% mdlGetTimeOfNextVarHit
% Return the time of the next hit for this block.  Note that the result is
% absolute time.  Note that this function is only used when you specify a
% variable discrete-time sample time [-2 0] in the sample time array in
% mdlInitializeSizes.
%=============================================================================
%
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

% end mdlGetTimeOfNextVarHit

%
%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(t,x,u)

sys = [];

% end mdlTerminate
