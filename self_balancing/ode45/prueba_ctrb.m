clc; clear; close all;

C = [ ...
     0     1.53     0      28.95;
     1.53  0       28.95  0;
     0    -3.84     0     -101.41;
    -3.84  0     -101.41   0 ];

rangoC = rank(C,1e-14)

A = [0 1 0 0;
     0 0 -7.54 0;
     0 0 0 1;
     0 0 26.41 0];

B = [0; 1.53; 0; -3.84];

rango = rank(ctrb(A,B))

s = svd(C)

condC = cond(C)

%%
Cn = C ./ vecnorm(C);
rankCn = rank(Cn)
sCn = svd(Cn)
condCn = cond(Cn)