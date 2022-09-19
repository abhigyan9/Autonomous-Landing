% online LQR gains calculator

function out = OL_LQR_maker(u,uav_P)

    A = u(1:16,1:16);
    B = u(1:16,17:20);
    C = u(17:28,1:16);
    D = u(17:28,17:20);

    r = uav_P.r;
    Q = uav_P.Q;
    R = uav_P.R;

    [K,S,e] = lqr(A,B,Q,R);

    out = [K(1,:),K(2,:),K(3,:),K(4,:)]';

end
