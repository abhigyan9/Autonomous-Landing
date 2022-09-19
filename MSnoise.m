function c=MSnoise(xx)

q = .01;

c = xx + sqrt(q)*randn(6,1);

