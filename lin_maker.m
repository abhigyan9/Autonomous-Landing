% Online Linearizer

% trimming around position or around velocity?

function out = lin_maker(u(4),uav_P)

x0 = u(1);
y0 = u(2);
z0 = u(3);
psi0 = u(4);

state0 = [x0;y0;z0;0;0;0;0;0;psi0;0;0;0;0;0;0;0];
input0 = [1;1;1;1];
output0 = state0;
% Specify initial guesses to keep
ix = [1,2,3,9]';
iu = [];
iy = ix;

[x_trim,u_trim,y_trim,dx_trim] = trim('c3_lining_model',state0,input0,output0,ix,iu,iy);

[A,B,C,D] = linmod('c3_lining_model',x_trim,u_trim);


