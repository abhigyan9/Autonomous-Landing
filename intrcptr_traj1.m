% MAE 275 Project
% ABHIGYAN | 30-5-22

% function to evaluate time optimal trajectory
% inputs - estimated UAV state[x12]; estimated MS states[x12]; current time;
%          previous time
% outputs - dynamically feasible desired trajectory to intercept position

function dstj = intrcptr_traj1(uu_UAV,uu_MS,t_c,t_prev)

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

    t1 = t_prev;
    t_c = t_c;
    del_t = t_c - t_prev;
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
    tao = sqrt((sum((uu_MS(1:3,1)-uu_UAV(1:3,1)).^2))/(sum((uu_MS(4:6,1)-uu_UAV(4:6,1)).^2)));
    t2 = t1 + tao;

    % calculate pn
    pn.given = [df_uav(1);df_ms(1);uu_UAV(1+3);uu_MS(1+3)];
    % for flat state 1
    t_mat = [t1^3, t1^2, t1, 1;...
           t2^3, t2^2, t2, 1;...
           3*t1^2, 2*t1, 1, 0;...
           3*t2^2, 2*t2, 1, 0];
    
    % direct inverse of pn.t_mat - helps reduce computational time
    %{
    pn.t_mat = [-2/(t1 - t2)^3, 2/(t1 - t2)^3, 1/(t1 - t2)^2, 1/(t1 - t2)^2;...
                (3*(t1 + t2))/(t1 - t2)^3, -(3*(t1 + t2))/(t1 - t2)^3, -(t1 + 2*t2)/(t1 - t2)^2, -(2*t1 + t2)/(t1 - t2)^2;...
                -(6*t1*t2)/(t1 - t2)^3, (6*t1*t2)/(t1 - t2)^3, (t2*(2*t1 + t2))/(t1 - t2)^2, (t1*(t1 + 2*t2))/(t1 - t2)^2;...
                (t2^2*(3*t1 - t2))/(t1 - t2)^3, (t1^2*(t1 - 3*t2))/(t1 - t2)^3, -(t1*t2^2)/(t1 - t2)^2, -(t1^2*t2)/(t1 - t2)^2];
    %}
    pn.coeff = inv(t_mat)*pn.given;
    
    % solving for pe
    pe.given = [df_uav(2);df_ms(2);uu_UAV(2+3);uu_MS(2+3)];
    pe.coeff = inv(t_mat)*pe.given;
    % solving for pd
    pd.given = [df_uav(3);df_ms(3);uu_UAV(3+3);uu_MS(3+3)];
    pd.coeff = inv(t_mat)*pd.given;
    % solving for psi
    psi.given = [uu_UAV(9);uu_MS(9);uu_UAV(9+3);uu_MS(9+3)];
    psi.coeff = inv(t_mat)*psi.given;

    flats(1) = pn.coeff(1)*t_c^3 + pn.coeff(2)*t_c^2 + pn.coeff(3)*t_c + pn.coeff(4);
    flats(2) = pe.coeff(1)*t_c^3 + pe.coeff(2)*t_c^2 + pe.coeff(3)*t_c + pe.coeff(4);
    flats(3) = pd.coeff(1)*t_c^3 + pd.coeff(2)*t_c^2 + pd.coeff(3)*t_c + pd.coeff(4);
    flats(4) = psi.coeff(1)*t_c^3 + psi.coeff(2)*t_c^2 + psi.coeff(3)*t_c + psi.coeff(4);    

    dstj(1) = flats(1);
    dstj(2) = flats(2);
    dstj(3) = flats(3);
    dstj(4) = 3*pn.coeff(1)*t_c^2 + 2*pn.coeff(2)*t_c + pn.coeff(3);
    dstj(5) = 3*pe.coeff(1)*t_c^2 + 2*pe.coeff(2)*t_c + pe.coeff(3);
    dstj(6) = 3*pd.coeff(1)*t_c^2 + 2*pd.coeff(2)*t_c + pd.coeff(3);
    
    beta_a = -cos(flats(4))*(6*pn.coeff(1)*t_c + 2*pn.coeff(2))...
            - sin(flats(4))*(6*pd.coeff(1)*t_c + 2*pd.coeff(2));
    beta_b = 9.81 - (6*pd.coeff(1)*t_c + 2*pd.coeff(2));
    beta_c = -sin(flats(4))*(6*pn.coeff(1)*t_c + 2*pn.coeff(2))...
            -cos(flats(4))*(6*pd.coeff(1)*t_c + 2*pd.coeff(2));
    
    dstj(7) = atan2(beta_a,beta_b);
    dstj(8) = atan2(beta_c,sqrt(beta_a^2 + beta_b^2));
    dstj(9) = flats(4);
    
    ct=cos(dstj(7)); cp=cos(dstj(8)); cs=cos(dstj(9));
    st=sin(dstj(7)); sp=sin(dstj(8)); ss=sin(dstj(9));
    Rot_mat = [ct*cs, sp*st*cs-cp*ss, cp*st*cs - sp*ss;...
               ct*ss, sp*st*ss+cp*cs, cp*st*ss - sp*cs;...
               -st, sp*ct, cp*ct];
    
    dstj(10:12) = 0;    % edit to implement Eqn7-part4
    dstj = dstj';
end

    









