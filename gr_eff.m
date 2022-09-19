% Includes ground effect, by modifying the Thrust force and roll, pitch
% torques

% input     ~ usual thrust and torques; separation btwn airframes
% outputs   ~ updated thrust force and torques

% primarily, ground effect is modeled as-
% F/F0 = 1/(1 - rho*(R/4Z)^2)
% tau_psi does not get affected yet....read more papers
% apart from Cg~plate surface separation, does relative pose also matter?

function out = gr_eff(fnt,delta_p,uav_P)
    
    fx           = fnt(1);
    fy           = fnt(2);
    fz           = fnt(3);
    tau_phi     = fnt(4);
    tau_theta   = fnt(5);
    tau_psi     = fnt(6);
    del_x = delta_p(1);
    del_y = delta_p(2);
    del_z = delta_p(3);

    R = uav_P.rotor_dia/2;
    rho = 0.8;    % coefficient to consider uncertain airflow due to multiple rotors

    if(abs(del_z) <= 4*R) && abs(del_y < 0.25) && (del_x < 0.25)
        fz = fz/(1-rho*(R/del_z)^2);
        tau_phi = tau_phi/(1-rho*(R/del_z)^2);
        tau_theta = tau_theta/(1-rho*(R/del_z)^2);
    end

    out = [fx;fy;fz;tau_phi;tau_theta;tau_psi];
end