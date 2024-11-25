clc
clear

% -------------

fig1 = figure(1);
clf
% fig1.Position(3:4) = [400 300];
xlabel('Position X (m)')
ylabel('Position Y (m)')
grid on
hold on
axis equal
% for x = 0:100:1000
%     for y = 0:100:1000
%         plot3(x,y,60,'.',Color='black')
%     end
% end



fig2 = figure(2);
clf
grid on

% fig2.Position(3:4) = [400 300];
xlabel('Time (s)')
ylabel('Velocity (m/s)')
axis([0 10 0 12])

% -------------
l  =  50;
z1 = 060;
z2 = 100;

posA = [ 000  000   z1 
           l  000   z2 
           l    l   z1 
         000    l   z2 ];
velA = [ 010  000  000 
         000  010  000
         -10  000  000 
         000  -10  000 ];

posB = [   l    l   z2 
         000    l   z1
         000  000   z2 
           l  000   z1 ];
velB = [ 000  010  000 
         -10  000  000
         000  -10  000 
         010  000  000 ];


wp1A = Waypoint;
wp1A.t   = 0;

wp1B = Waypoint;
wp1B.t   = 10;

fp1  = FlightPlan(Waypoint.empty);
fp1.SetWaypoint(wp1A);
fp1.SetWaypoint(wp1B);


% -------------

for i = 1:4

    wp1A.pos = posA(i,:);
    wp1A.vel = velA(i,:);
    wp1B.pos = posB(i,:);
    wp1B.vel = velB(i,:);

    fp1.SetJLS()
    tr = fp1.Trace(0.01);
    tr = tr(1:end-1,:);

    % pos
    figure(1)
    hold on
    plot3(tr(:,2),tr(:,3),tr(:,4), ...
        '-', ...
        LineWidth = 1.5 );

    wp1A.PlotVel;
    wp1B.PlotVel;
    

    % vel
    figure(2)
    hold on
    plot(tr(:,1),sqrt(tr(:,5).^2 + tr(:,6).^2 + tr(:,7).^2), ...
        '-', ...
        LineWidth = 1.5 )


end





% figure(1)
% legend('10s', '12s' ,'14s','16s','18s','20s',Location='northwest') 
% 
% figure(2)
% legend('10s', '12s' ,'14s','16s','18s','20s',Location='northeast') 
% 



