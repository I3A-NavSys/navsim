clc;clear

% r  = 0;
% v  = 10;
% 
% v2 = 12;
% 
% t = 3;

syms r v v2 t

syms d1 d2 r2
eq = ...
[
    r2 == r + v *t +            + 1/6*d1*t^3 + 1/24*d2*t^4 ;
    v2 == v +      + 1/2*d1*t^2 + 1/6*d2*t^3 ;
    0  ==     d1*t + 1/2*d2*t^2 ;
];

sol = solve(eq,r2,d1,d2)
% d1  = eval(sol.d1);
% d2  = eval(sol.d2);
r2  = eval(sol.r2);


t = 0:0.01:t;
rr = r + v .*t + 1/6*d1*t.^3 + 1/24*d2*t.^4 ;
vv = v + 1/2*d1*t.^2 + 1/6*d2*t.^3  ;
aa = d1.*t + 1/2*d2*t.^2 ;

[ rr(end)  vv(end)  aa(end) ]


figure(1)
plot(t,rr)
grid on

figure(2)
plot(t,vv)
grid on

figure(3)
plot(t,aa)
grid on

