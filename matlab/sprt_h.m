epsn = 0.1155
eps = 0.1
del = 0.01785

f = @(h) epsn * (del/eps) ^ h + (1-epsn) * ((1 - del)/(1 - eps)) ^ h - 1;
df = @(h) epsn * (del/eps) ^ h + (1-epsn) * ((1 - del)/(1 - eps)) ^ h;

xn_1 = 2;
iter = 0
while abs (f(xn_1)) > 0.0001
    xn = xn_1 - f(xn_1)/df(xn_1);
    xn_1 = xn;
    iter = iter + 1
end
xn
