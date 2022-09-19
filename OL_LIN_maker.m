% Online Linearizer

% trimming around position or around velocity?

function out = OL_LIN_maker(u,uav_P)

x0 = u(1);
y0 = u(2);
z0 = u(3);
psi0 = u(4);
u0 = u(5);
v0 = u(6);
w0 = u(9);

state0 = [x0;y0;z0;u0;v0;w0;0;0;psi0;0;0;0;0;0;0;0];
input0 = [1;1;1;1];
output0 = state0;
% Specify initial guesses to keep
ix = [1,2,3,4,5,6,9];
iu = [];
iy = ix;

[x_trim,u_trim,y_trim,dx_trim] = trim('c3_lining_model',state0,input0,output0,ix,iu,iy);

[A,B,C,D] = linmod('c3_lining_model',x_trim,u_trim);
%r = uav_P.r;
Q = uav_P.Q;
R = uav_P.R;

[K,S,e] = lqr(A,B,Q,R);

out = [K(1,:),K(2,:),K(3,:),K(4,:)]';

end
