% HW3 PART 3 PARAM FILE

uav_P.gravity = 9.81;
uav_P.Ts = 0.01;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Params for Aersonade UAV
%physical parameters of airframe

uav_P.mass = 0.3815;
uav_P.Jx   = 2.3951E-5;
uav_P.Jy   = 2.3951E-5;
uav_P.Jz   = 3.2347E-5;
% P.Jxz  = 0.175; % from discord ~ recordings

uav_P.k1 = 5;   % rotor thrust constant
uav_P.k2 = 5;   % rotor torque constant
uav_P.T = 1;   % motor torque constant
uav_P.chl = 1;  % characteristic length of MAV ~ inter-rotor distance
% above data from week3class1

uav_P.J_rotor = 0.01;
uav_P.rotor_dia = 0.2;     % rotor diameter ~ for ground effect

% initial conditions
uav_P.pn0    = 1;  % initial North position
uav_P.pe0    = 1;  % initial East position
uav_P.pd0    = 1;  % initial Down position (negative altitude)
uav_P.u0     = 0;  % initial velocity along body x-axis
uav_P.v0     = 0;  % initial velocity along body y-axis
uav_P.w0     = 0;  % initial velocity along body z-axis
uav_P.phi0   = 0;  % initial roll angle
uav_P.theta0 = 0;  % initial pitch angle
uav_P.psi0   = 0;  % initial yaw angle
uav_P.p0     = 0;  % initial body frame roll rate
uav_P.q0     = 0;  % initial body frame pitch rate
uav_P.r0     = 0;  % initial body frame yaw rate

uav_P.omega_f0 = 0.18712575;
uav_P.omega_r0 = 0.18712575;
uav_P.omega_b0 = 0.18712575;
uav_P.omega_l0 = 0.18712575;

%uav_P.omega_f0 = 0;
%uav_P.omega_r0 = 0;
%uav_P.omega_b0 = 0;
%uav_P.omega_l0 = 0;

% steady ambient wind
uav_P.wind_n = 0;
uav_P.wind_e = 0;
uav_P.wind_d = 0;

x0_ms.pn0 = 20;
x0_ms.pe0 = 20;
x0_ms.pd0 = 20;

uav_P.r = 20;
uav_P.Q = diag([1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]);
uav_P.R = uav_P.r*diag([1,1,1,1]);

uav_P.Nos = 0.00001;

uav_P.QQ = sqrt(uav_P.Nos)*eye(16);
uav_P.RR = sqrt(uav_P.Nos)*eye(12);
uav_P.INIT = [uav_P.pn0,  uav_P.pe0,    uav_P.pd0,   uav_P.u0, uav_P.v0, ...
               uav_P.w0, uav_P.phi0, uav_P.theta0, uav_P.psi0, uav_P.p0, ...
               uav_P.q0, uav_P.r0, ...
               uav_P.omega_f0, uav_P.omega_r0, uav_P.omega_b0,uav_P.omega_l0];
