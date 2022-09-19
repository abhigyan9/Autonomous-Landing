% Linearizing the Non-Linear UAV Dynamics block


x0 = [1;1;1;0;0;0;0;0;0;0;0;0;0;0;0;0];
u0 = [1;1;1;1];
y0 = x0;
% Specify initial guesses to keep
ix = [1,2,3,4,5,6,7,8,9,10,11,12]';
iu = [];
iy = ix;

[x_trim,u_trim,y_trim,dx_trim] = trim('c3_lining_model',x0,u0,y0,ix,iu,iy);

[A,B,C,D] = linmod('c3_lining_model',x_trim,u_trim);

r = 20;
Q = diag([1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]);
R = r*diag([1,1,1,1]);

% lqr
[K,S,e] = lqr(A,B,Q,R);

% saving parameters of designed controller

uav_P.A = A;
uav_P.B = B;
uav_P.C = C;
uav_P.D = D;
uav_P.K_LQR = K;
uav_P.omega_f0 = x_trim(13);
uav_P.omega_r0 = x_trim(14);
uav_P.omega_b0 = x_trim(15);
uav_P.omega_l0 = x_trim(16);
