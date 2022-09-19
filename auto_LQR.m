function y = auto_LQR(uu,uav_P)
%
% Linearized controller


    %% Process inputs
    NN = 0; % states
    x       = uu(1+NN);  % inertial North position
    y       = uu(2+NN);  % inertial East position
    z       = uu(3+NN);  % inertial Down position
    u        = uu(4+NN);  % body x velocity
    v        = uu(5+NN);  % body y velocity
    w        = uu(6+NN);  % body z velocity
    phi      = uu(7+NN);  % roll angle
    theta    = uu(8+NN);  % pitch angle
    psi      = uu(9+NN);  % yaw angle
    p        = uu(10+NN); % body frame roll rate
    q        = uu(11+NN); % body frame pitch rate
    r        = uu(12+NN); % body frame yaw rate
    om_f = uu(13+NN);
    om_r = uu(14+NN);
    om_b = uu(15+NN);
    om_l = uu(16+NN);
    NN = NN+16; % desired flat outputs
    xx_e     = uu(1+NN:16+NN);  % desired state
    NN = NN+16;
    t        = uu(1+NN);   % time
    OL_LQR = uu((2+NN):end)';  % online lqr gain
    OL_LQR = [OL_LQR(1:16);OL_LQR(17:32);OL_LQR(33:48);OL_LQR(49:64)];

    %% Controller
    
    %
    if t > 1
        K = OL_LQR;
    else
        K = uav_P.K_LQR;
    end

    % LQR controller
    x_c = uu(1:16,1);   % last sensed state
    u = -K*(x_c-xx_e)   % current control law
    
    %% create outputs
    
    % control outputs

    y = u;
 
end
