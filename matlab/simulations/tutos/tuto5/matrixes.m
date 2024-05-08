clc;  clear

r  = [  0  0  0 ];
v  = [  2  0  0 ];
a  = [  0  0  0 ];

r2 = [ 10 10  0 ];
v2 = [  0  2  0 ];
a2 = [  0  0  0 ];

t = 8;

A = [ t^3/6   t^4/24   t^5/120 
      t^2/2   t^3/6    t^4/24 
      t       t^2/2    t^3/6  ];

B = [ r2-r-v*t 
      v2-v-a*t  
      a2-a      ];

if rank(A) < 3
    error('Error. Interpolación TPV sin solución')
end  

X = A\B;
x1  = X(1,:);  % jerk
x2  = X(2,:);  % jolt
x3  = X(3,:);  % jolt_dot


tt = 0:0.01:t;
rr = zeros(length(tt),3);
vv = zeros(length(tt),3);
aa = zeros(length(tt),3);

for i = 1:length(tt)
    t = tt(i);
    rr(i,:) = r +  v*t + 1/2* a*t^2 + 1/6*x1*t^3 + 1/24*x2*t^4 + 1/120*x3*t^5 ;
    vv(i,:) = v +  a*t + 1/2*x1*t^2 + 1/6*x2*t^3 + 1/24*x3*t^4 ;
    aa(i,:) = a + x1*t + 1/2*x2*t^2 + 1/6*x3*t^3 ;
end

rr(end,:)
vv(end,:)
aa(end,:)


figure(1)
plot(tt,rr)
grid on

figure(2)
plot(tt,vv)
grid on

figure(3)
plot(tt,aa)
grid on


figure(4)
plot3(rr(:,1),rr(:,2),rr(:,3),LineWidth=2)
axis([-10 10 -10 10 -10 10])
grid on
xlabel('X')
ylabel('Y')
zlabel('Z')

viscircles([0 10],10)
