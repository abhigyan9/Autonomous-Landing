% Given 2 state points, can we get dynamically feasible trajectory out of
% them?
% point 1
st1 = [0;0;0;5;4;3;0;0;pi/6;0;0;0];
st2 = [30;40;50;1;1;2;0;0;pi/6;0;0;0];

flat0 = [st1(1:3,1);st1(9,1)];
flat_f = [st2(1:3,1);st2(9,1)];

rel_dist = sqrt(sum((st2(1:3,1)-st1(1:3,1)).^2));
rel_v = sqrt(sum((st2(4:6,1)-st1(4:6,1)).^2));
tao = rel_dist/rel_v;

t1 = 0;
t2 = t1 + tao;
del_t = 0.1;

%% solving for x
px.given = [st1(1);st2(1);st1(4);st2(4)];
px.t_mat = [t1^3, t1^2, t1, 1;...
           t2^3, t2^2, t2, 1;...
           3*t1^2, 2*t1, 1, 0;...
           3*t2^2, 2*t2, 1, 0];
px.coeff = inv(px.t_mat)*px.given;
t = linspace(t1,t2,(t2-t1)/del_t);  % can be replaced by a non-linear time space to reduce computation time
x = px.coeff(1)*t.^3 + px.coeff(2)*t.^2 + px.coeff(3)*t + px.coeff(4);
plot(t,x)
hold on

tao = tao/2;
t2 = t1 + tao;
del_t = 0.1;

%% solving for x
px.given = [st1(1);st2(1);st1(4);st2(4)];
px.t_mat = [t1^3, t1^2, t1, 1;...
           t2^3, t2^2, t2, 1;...
           3*t1^2, 2*t1, 1, 0;...
           3*t2^2, 2*t2, 1, 0];
px.coeff = inv(px.t_mat)*px.given;
t = linspace(t1,t2,(t2-t1)/del_t);  % can be replaced by a non-linear time space to reduce computation time
x = px.coeff(1)*t.^3 + px.coeff(2)*t.^2 + px.coeff(3)*t + px.coeff(4);
plot(t,x)
hold on

tao = tao/2;
t2 = t1 + tao;
del_t = 0.1;

%% solving for x
px.given = [st1(1);st2(1);st1(4);st2(4)];
px.t_mat = [t1^3, t1^2, t1, 1;...
           t2^3, t2^2, t2, 1;...
           3*t1^2, 2*t1, 1, 0;...
           3*t2^2, 2*t2, 1, 0];
px.coeff = inv(px.t_mat)*px.given;
t = linspace(t1,t2,(t2-t1)/del_t);  % can be replaced by a non-linear time space to reduce computation time
x = px.coeff(1)*t.^3 + px.coeff(2)*t.^2 + px.coeff(3)*t + px.coeff(4);
plot(t,x)
hold on
