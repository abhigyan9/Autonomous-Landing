% MAE 275 Project
% ABHIGYAN | 30-5-22

% function to evaluate time optimal trajectory
% inputs - estimated UAV state[x12]; estimated MS states[x12]; current time;
%          previous time
% outputs - dynamically feasible desired trajectory to intercept position

function dstj = intrcptr_traj2(uu_UAV,uu_MS,t_c,t_prev)

%% unfold inputs
    uav.pn    = uu_UAV(1);
    uav.pe    = uu_UAV(2);
    uav.pd    = uu_UAV(3);
    uav.u     = uu_UAV(4);
    uav.v     = uu_UAV(5);
    uav.w     = uu_UAV(6);
    uav.phi   = uu_UAV(7);
    uav.theta = uu_UAV(8);
    uav.psi   = uu_UAV(9);
    uav.p     = uu_UAV(10);
    uav.q     = uu_UAV(11);
    uav.r     = uu_UAV(12);

    ms.pn    = uu_MS(1);
    ms.pe    = uu_MS(2);
    ms.pd    = uu_MS(3);
    ms.u     = uu_MS(4);
    ms.v     = uu_MS(5);
    ms.w     = uu_MS(6);
    ms.phi   = uu_MS(7);
    ms.theta = uu_MS(8);
    ms.psi   = uu_MS(9);
    ms.p     = uu_MS(10);
    ms.q     = uu_MS(11);
    ms.r     = uu_MS(12);

    if t_prev == 0
        t1 = t_c;
    else
        t1 = t_prev;
    end
    
%% get relative positions
    df_uav(1) = uu_UAV(1);
    df_uav(2) = uu_UAV(2);
    df_uav(3) = uu_UAV(3);
    df_uav(4) = uu_UAV(9);
    df_ms(1) = uu_MS(1);
    df_ms(2) = uu_MS(2);
    df_ms(3) = uu_MS(3);
    df_ms(4) = uu_MS(9);
    % tao = relative distance / relative velocity; Ensures < v_max
    if  sum((uu_MS(4:6,1)-uu_UAV(4:6,1)).^2) == 0
        tao = 0.2*sqrt((sum((uu_MS(1:3,1)-uu_UAV(1:3,1)).^2)));
    else
        tao = sqrt(sum((uu_MS(1:3,1)-uu_UAV(1:3,1)).^2))/...
            (sqrt(sum((uu_UAV(4:6,1)).^2))-sqrt(sum(uu_MS(4:6,1)).^2));
    end
    if tao < 0
        tao = 10;
    end

    t2 = t1 + tao;

    % calculate pn
    pn.given = [df_uav(1);df_ms(1);uu_UAV(1+3);uu_MS(1+3)];
    % for flat state 1
    t_mat = [t1^3, t1^2, t1, 1;...
           t2^3, t2^2, t2, 1;...
           3*t1^2, 2*t1, 1, 0;...
           3*t2^2, 2*t2, 1, 0];
    inv_t_mat = inv(t_mat);

    % direct inverse of pn.t_mat - helps reduce computational time
    %{
    inv_t_mat = [-2/(t1 - t2)^3, 2/(t1 - t2)^3, 1/(t1 - t2)^2, 1/(t1 - t2)^2;...
                (3*(t1 + t2))/(t1 - t2)^3, -(3*(t1 + t2))/(t1 - t2)^3, -(t1 + 2*t2)/(t1 - t2)^2, -(2*t1 + t2)/(t1 - t2)^2;...
                -(6*t1*t2)/(t1 - t2)^3, (6*t1*t2)/(t1 - t2)^3, (t2*(2*t1 + t2))/(t1 - t2)^2, (t1*(t1 + 2*t2))/(t1 - t2)^2;...
                (t2^2*(3*t1 - t2))/(t1 - t2)^3, (t1^2*(t1 - 3*t2))/(t1 - t2)^3, -(t1*t2^2)/(t1 - t2)^2, -(t1^2*t2)/(t1 - t2)^2];
    %}
    pn.coeff = inv_t_mat*pn.given;
    
    % solving for pe
    pe.given = [df_uav(2);df_ms(2);uu_UAV(2+3);uu_MS(2+3)];
    pe.coeff = inv_t_mat*pe.given;
    % solving for pd
    pd.given = [df_uav(3);df_ms(3);uu_UAV(3+3);uu_MS(3+3)];
    pd.coeff = inv_t_mat*pd.given;
    % solving for psi
    %psi.given = [uu_UAV(9);uu_MS(9);uu_UAV(9+3);uu_MS(9+3)];
    %psi.coeff = inv_t_mat*psi.given;

    if tao < 2.5
        t_s = t_c;
    else
        t_s = t1+0.5*(t2-t1);
    end
    flats(1) = pn.coeff(1)*t_s^3 + pn.coeff(2)*t_s^2 + pn.coeff(3)*t_s + pn.coeff(4);
    flats(2) = pe.coeff(1)*t_s^3 + pe.coeff(2)*t_s^2 + pe.coeff(3)*t_s + pe.coeff(4);
    flats(3) = pd.coeff(1)*t_s^3 + pd.coeff(2)*t_s^2 + pd.coeff(3)*t_s + pd.coeff(4);
    %flats(4) = psi.coeff(1)*t_s^3 + psi.coeff(2)*t_s^2 + psi.coeff(3)*t_s + psi.coeff(4);    
    flats(5) = 3*pn.coeff(1)*t_s^2 + 2*pn.coeff(2)*t_s + pn.coeff(3);
    flats(6) = 3*pe.coeff(1)*t_s^2 + 2*pe.coeff(2)*t_s + pn.coeff(3);
    flats(7) = 3*pd.coeff(1)*t_s^2 + 2*pd.coeff(2)*t_s + pn.coeff(3);
    
    state0 = [flats(1);flats(2);flats(3);flats(5);flats(6);flats(7);0;0;0;0;0;0;0;0;0;0];
    input0 = [1;1;1;1];
    output0 = state0;
    % Specify initial guesses to keep
    ix = [1,2,3,4,5,6];
    iu = [];
    iy = ix;

    [x_trim,u_trim,y_trim,dx_trim] = trim('c3_lining_model',state0,input0,output0,ix,iu,iy);

    dstj = [(x_trim);(u_trim)];
    %dstj = [flats(1);flats(2);flats(3);0;0;0;0;0;0;0;0;0;0.18712575;0.18712575;0.18712575;0.18712575];
end

    









