% rotor dynamics s-function file to get thrust force and torque

% states    ~ 4 - rotor angular velocities
% input     ~ 12,4 - [UAV_states; motor currents]
% outputs   ~ 6 - fx,fy,fz and roll,pitch,yaw torques 

function [sys,x0,str,ts,simStateCompliance] = rotor_dynamics(t,x,uu,flag,uav_P)
%SFUNTMPL General MATLAB S-Function Template
%   With MATLAB S-functions, you can define you own ordinary differential
%   equations (ODEs), discrete system equations, and/or just about
%   any type of algorithm to be used within a Simulink block diagram.
%
%   The general form of an MATLAB S-function syntax is:
%       [SYS,X0,STR,TS,SIMSTATECOMPLIANCE] = SFUNC(T,X,U,FLAG,P1,...,Pn)
%
%   What is returned by SFUNC at a given point in time, T, depends on the
%   value of the FLAG, the current state vector, X, and the current
%   input vector, U.
%
%   FLAG   RESULT             DESCRIPTION
%   -----  ------             --------------------------------------------
%   0      [SIZES,X0,STR,TS]  Initialization, return system sizes in SYS,
%                             initial state in X0, state ordering strings
%                             in STR, and sample times in TS.
%   1      DX                 Return continuous state derivatives in SYS.
%   2      DS                 Update discrete states SYS = X(n+1)
%   3      Y                  Return outputs in SYS.
%   4      TNEXT              Return next time hit for variable step sample
%                             time in SYS.
%   5                         Reserved for future (root finding).
%   9      []                 Termination, perform any cleanup SYS=[].
%
%
%   The state vectors, X and X0 consists of continuous states followed
%   by discrete states.
%
%   Optional parameters, P1,...,Pn can be provided to the S-function and
%   used during any FLAG operation.
%
%   When SFUNC is called with FLAG = 0, the following information
%   should be returned:
%
%      SYS(1) = Number of continuous states.
%      SYS(2) = Number of discrete states.
%      SYS(3) = Number of outputs.
%      SYS(4) = Number of inputs.
%               Any of the first four elements in SYS can be specified
%               as -1 indicating that they are dynamically sized. The
%               actual length for all other flags will be equal to the
%               length of the input, U.
%      SYS(5) = Reserved for root finding. Must be zero.
%      SYS(6) = Direct feedthrough flag (1=yes, 0=no). The s-function
%               has direct feedthrough if U is used during the FLAG=3
%               call. Setting this to 0 is akin to making a promise that
%               U will not be used during FLAG=3. If you break the promise
%               then unpredictable results will occur.
%      SYS(7) = Number of sample times. This is the number of rows in TS.
%
%
%      X0     = Initial state conditions or [] if no states.
%
%      STR    = State ordering strings which is generally specified as [].
%
%      TS     = An m-by-2 matrix containing the sample time
%               (period, offset) information. Where m = number of sample
%               times. The ordering of the sample times must be:
%
%               TS = [0      0,      : Continuous sample time.
%                     0      1,      : Continuous, but fixed in minor step
%                                      sample time.
%                     PERIOD OFFSET, : Discrete sample time where
%                                      PERIOD > 0 & OFFSET < PERIOD.
%                     -2     0];     : Variable step discrete sample time
%                                      where FLAG=4 is used to get time of
%                                      next hit.
%
%               There can be more than one sample time providing
%               they are ordered such that they are monotonically
%               increasing. Only the needed sample times should be
%               specified in TS. When specifying more than one
%               sample time, you must check for sample hits explicitly by
%               seeing if
%                  abs(round((T-OFFSET)/PERIOD) - (T-OFFSET)/PERIOD)
%               is within a specified tolerance, generally 1e-8. This
%               tolerance is dependent upon your model's sampling times
%               and simulation time.
%
%               You can also specify that the sample time of the S-function
%               is inherited from the driving block. For functions which
%               change during minor steps, this is done by
%               specifying SYS(7) = 1 and TS = [-1 0]. For functions which
%               are held during minor steps, this is done by specifying
%               SYS(7) = 1 and TS = [-1 1].
%
%      SIMSTATECOMPLIANCE = Specifices how to handle this block when saving and
%                           restoring the complete simulation state of the
%                           model. The allowed values are: 'DefaultSimState',
%                           'HasNoSimState' or 'DisallowSimState'. If this value
%                           is not speficified, then the block's compliance with
%                           simState feature is set to 'UknownSimState'.


%   Copyright 1990-2010 The MathWorks, Inc.

%
% The following outlines the general structure of an S-function.
%
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
    sys=mdlDerivatives(t,x,uu,uav_P);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2,
    sys=mdlUpdate(t,x,uu);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,uu,uav_P);

  %%%%%%%%%%%%%%%%%%%%%%%
  % GetTimeOfNextVarHit %
  %%%%%%%%%%%%%%%%%%%%%%%
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,uu);

  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9,
    sys=mdlTerminate(t,x,uu);

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

sizes.NumContStates  = 4;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 10;
sizes.NumInputs      = 16;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

%
% initialize the initial conditions
%
x0  = [uav_P.omega_f0; uav_P.omega_r0; uav_P.omega_b0; uav_P.omega_l0];

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
function sys=mdlDerivatives(t,x,uu,uav_P)

    omega_f = x(1);
    omega_r = x(2);
    omega_b = x(3);
    omega_l = x(4);
    i_f = uu(1);
    i_r = uu(2);
    i_b = uu(3);
    i_l = uu(4);
    k1 = uav_P.k1;
    k2 = uav_P.k2;
    T = uav_P.T;
    chl = uav_P.chl;
    J_rotor = uav_P.J_rotor;

    omega_f_dot = (-k2*omega_f + T*i_f)/J_rotor;
    omega_r_dot = (-k2*omega_r + T*i_r)/J_rotor;
    omega_b_dot = (-k2*omega_b + T*i_b)/J_rotor;
    omega_l_dot = (-k2*omega_l + T*i_l)/J_rotor;

    % improved, cited model state derivatives
    %{
    T_e(1) = K_e_motor*i_motor(1)/omega(1); % Motor Torque
    % K_e_motor is the electrical torque constant = efficiency*voltage
    T_prop(1) = 4*rho_air*Cp*(omega(1)^2)*(prop_rad^5)/(pi^3);  % Aero
    Torque
    % Cp is coefficient of power, can be assumed constant?
    omega_dot(1) = (T_e(1) - T_prop(1) - B*omega(1))/(J_mnr);
    %}
    
sys = [omega_f_dot; omega_r_dot; omega_b_dot; omega_l_dot];

% end mdlDerivatives

%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================
%
function sys=mdlUpdate(t,x,uu)

sys = [];

% end mdlUpdate

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,uu,uav_P)

    omega_f = x(1);
    omega_r = x(2);
    omega_b = x(3);
    omega_l = x(4);
    pn    = uu(4+1);
    pe    = uu(4+2);
    pd    = uu(4+3);
    u_vel     = uu(4+4);
    v     = uu(4+5);
    w     = uu(4+6);
    phi   = uu(4+7);
    theta = uu(4+8);
    psi   = uu(4+9);
    p     = uu(4+10);
    q     = uu(4+11);
    r     = uu(4+12);

    k1 = uav_P.k1;
    k2 = uav_P.k2;
    T = uav_P.T;
    chl = uav_P.chl;
    J_rotor = uav_P.J_rotor;
    mass = uav_P.mass;
    grav = uav_P.gravity;

    % fx = -mass*grav*sin(theta);
    % fy = mass*grav*cos(theta)*sin(phi);
    % fz = mass*grav*cos(theta)*cos(phi) - k1*(omega_f+omega_r+omega_b+omega_l);
    fx = 0;
    fy = 0;
    % Rotor thrust including density effectx
        %fz = -k1*(omega_f+omega_r+omega_b+omega_l)*exp(-grav*0.0289644*pd/(8.31445*288.15));
        fz = - k1*(omega_f+omega_r+omega_b+omega_l);
    tau_phi = (omega_l-omega_r)*chl*k1;
    tau_theta = (omega_f-omega_b)*chl*k1;
    tau_psi = k2*(-omega_f+omega_r-omega_b+omega_l);

    % improved cited model thrust and torques
    %{
    fz = 4*rho_air*Ct*(omega(1)^2+omega(2)^2+omega(3)^2+omega(4)^2)...
    *(prop_rad^4)/(pi^3);  % Aero thrust
    % Ct is coefficient of thrust, can be assumed constant?
    % Aero Torques
    tau_phi = chl*4*rho_air*Ct*(omega(4)^2 - omega(2)^2)*(prop_rad^4)/(pi^3);
    tau_theta = chl*4*rho_air*Ct*(omega(1)^2 - omega(3)^2)*(prop_rad^4)/(pi^3);
    tau_psi = 4*rho_air*Cp*(-omega(1)^2+omega(2)^2-omega(3)^2+omega(4)^2)...
    *(prop_rad^5)/(pi^3);
    % Cp is coefficient of power, can be assumed constant?
    %}

sys = [omega_f; omega_r; omega_b; omega_l; fx; fy; fz; tau_phi; tau_theta; tau_psi];

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
