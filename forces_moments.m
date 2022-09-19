% forces_moments.m
%   Computes the forces and moments acting on the airframe. 
%
%   Output is
%       F     - forces
%       M     - moments

function out = forces_moments(x, delta, wind, P)

    % relabel the inputs
    pn      = x(1);
    pe      = x(2);
    pd      = x(3);
    u       = x(4);
    v       = x(5);
    w       = x(6);
    phi     = x(7);
    theta   = x(8);
    psi     = x(9);
    p       = x(10);
    q       = x(11);
    r       = x(12);
    omega_f = delta(1);
    omega_r = delta(2);
    omega_b = delta(3);
    omega_l = delta(4);
    w_ns    = wind(1); % steady wind - North
    w_es    = wind(2); % steady wind - East
    w_ds    = wind(3); % steady wind - Down
    u_wg    = wind(4); % gust along body x-axis
    v_wg    = wind(5); % gust along body y-axis    
    w_wg    = wind(6); % gust along body z-axis
    
    % rotation matrix from vehicle to body
    %{
    sp = sin(phi);
    cp = cos(phi);
    st = sin(theta);
    ct = cos(theta);
    ss = sin(psi);
    cs = cos(psi);
    %}
    
    k1 = P.k1;      % motor thrust constant
    k2 = P.k2;      % motor torque constant
    chl = P.chl;    % characteristic length for drone ~ rotor arm diameter
    
    %{
    rotation_to_body = [ct*cs ct*ss -st;
                        sp*st*cs-cp*ss sp*st*ss+cp*cs sp*ct;
                        cp*st*cs+sp*ss cp*st*ss-sp*cs cp*ct
                        ];
    
    V_wind_body = rotation_to_body * [w_ns; w_es; w_ds] + [u_wg; v_wg; w_wg];
    V_airspeed_body = [u-V_wind_body(1); v-V_wind_body(2); w-V_wind_body(3)];
                    
    % compute wind data in NED
    w_n = 0;
    w_e = 0;
    w_d = 0;
    
    % compute air data
    u_r = V_airspeed_body(1);
    v_r = V_airspeed_body(2);
    w_r = V_airspeed_body(3);
    % Va = sqrt(u_r^2 + v_r^2 + w_r^2);     % air speed
    % alpha = atan(w_r / u_r);  % angle of attack
    % beta = asin(v_r / (Va));   % side slip
    %}

    omega_f_dot = (-k2*omega_f + T*i_f)/J_rotor;
    omega_r_dot = (-k2*omega_r + T*i_r)/J_rotor;
    omega_b_dot = (-k2*omega_b + T*i_b)/J_rotor;
    omega_l_dot = (-k2*omega_l + T*i_l)/J_rotor;

    F = k_1*(omega_f+omega_r+omega_b+omega_l);
    tau_phi = (omega_l-omega_r)*chl*k1;
    tau_theta = (omega_f-omega_b)*chl*k1;
    tau_psi = k2*(omega_f+omega_r+omega_b+omega_l);

    % compute external forces and torques on aircraft
    
    coeff_mat = [k1,      k1,      k1,     k1;...
                 0,       -chl*k1, 0,      chl*k1;...
                 chl*k1,  0,      -chl*k1, 0;...
                -chl*k2,  chl*k2, -chl*k2, chl*k2];

    F_n_torques = coeff_mat*[omega_f;omega_r;omega_b;omega_l];

    out = F_n_torques;
end