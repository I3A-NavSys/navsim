clc;clear

% r  = 0;
% v  = 5;
% a  = 0;
% 
% r2 = 10;
% v2 = 2;
% a2 = 0;

% t = 2.856;

syms r v a r2 v2 a2
syms j l t
eq = ...
[
    r2 == r + v*t + 1/2*a*t^2 + 1/6*j*t^3 + 1/24*l*t^4 ;
    v2 == v + a*t + 1/2*j*t^2 + 1/6*l*t^3 ;
    a2 == a + j*t + 1/2*l*t^2 ;
];

sol = solve(eq,[j l t]);
j  = eval(sol.j);
l  = eval(sol.l);
s  = eval(sol.s);


t = 0:0.01:t;
rr = r + v.*t + 1/2*a*t.^2 + 1/6*j*t.^3 + 1/24*l*t.^4 + 1/120*s*t.^5 ;
vv = v + a.*t + 1/2*j*t.^2 + 1/6*l*t.^3 + 1/24*s*t.^4 ;
aa = a + j.*t + 1/2*l*t.^2 + 1/6*s*t.^3 ;

[ rr(end)  vv(end)  aa(end) ];


figure(1)
plot(t,rr)
grid on

figure(2)
plot(t,vv)
grid on

figure(3)
plot(t,aa)
grid on
