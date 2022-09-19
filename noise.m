function d=noise(xx)

q = .001;
N = sqrt(q)*randn(1);
QQQ = zeros(10,1);
QQ = [N;N;QQQ];

d = xx + QQ;