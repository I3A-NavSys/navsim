clc;  clear

% r  = [  0  0  0 ];
% v  = [ 10  0  0 ];
% a  = [  0  0  0 ];
% 
% r2 = [ 100 100  0 ];
% v2 = [   0  10  0 ];
% a2 = [   0   0  0 ];
% 
% t = 13.2;
% t = 100 * pi/2 / 10    % 15.7
% t = 16.45;
% t = 20;



r  = [  0  0  0 ];
v  = [ 10  0  0 ];
a  = [  0  0  0 ];

r2 = [ 100   0   0 ];
v2 = [   5   0   0 ];
a2 = [   0   0   0 ];

t = 13.2;
t = 100 * pi/2 / 10    % 15.7
t = 16;
t = 13.3;
% t = 15







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

% rr(end,:)
% vv(end,:)
% aa(end,:)


figure(1)
clf
plot(tt,rr)
grid on

figure(2)
clf
plot(tt,vv)
hold on
plot(tt,sqrt(vv(:,1).^2 + vv(:,2).^2 + vv(:,3).^2),LineWidth=2,Color='k')
grid on

figure(3)
clf
plot(tt,aa)
grid on


figure(4)
clf
viscircles([0 100],100, LineWidth=1);
hold on
plot3(rr(:,1),rr(:,2),rr(:,3),LineWidth=2)
grid on
xlabel('X')
ylabel('Y')
zlabel('Z')

axis equal
% axis([0 100 0 100 -10 90])
view(2)
