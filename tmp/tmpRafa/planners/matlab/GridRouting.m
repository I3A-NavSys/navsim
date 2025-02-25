clc
clear

% -------------


z0 = 020;
z1 = 060;
z2 = 100;
% z2 = 101;



% -------------

fig1 = figure(1);
clf
% fig1.Position(3:4) = [900 900];
xlabel('Position X (m)')
ylabel('Position Y (m)')
grid on
hold on
axis([0 1000 0 1000 0 200])
% axis equal





% % Coordenadas de los vértices del rectángulo
% x = [0 1 1 0]; % Coordenadas x de los vértices
% y = [0 0 1 1]; % Coordenadas y de los vértices
% z = [0 0 0 0]; % Coordenadas z (plano horizontal)
% % Dibujar el rectángulo en 3D
% fill3(x, y, z, 'r'); % 'r' para color rojo
% 
% % Añadir un rectángulo en otro plano (opcional)
% z2 = [1 1 1 1]; % Cambia la altura a z=1
% fill3(x, y, z2, 'b'); % 'b' para color azul






colorZ1 = [0.8500 0.3250 0.0980];
colorZ2 = [0.9290 0.6940 0.1250];

for y = 0:100:1000
    for x = 50:100:950
        plot3(x,y,z1,'o', ...
              MarkerSize=4, MarkerFaceColor = colorZ1,...
              Color = colorZ1)
        if mod(y/100,2) == 0
            quiver3(x,y,z1, 100,0,0,':', ...
                    LineWidth = 1,   ...
                    MaxHeadSize = 10, ...
                    Color=colorZ1)
        else
            quiver3(x+100,y,z1, -100,0,0,':', ...
                    LineWidth = 1,   ...
                    MaxHeadSize = 2, ...
                    Color=colorZ1)
        end
    end
end

for x = 0:100:1000
    for y = 50:100:950
        plot3(x,y,z2,'o', ...
              MarkerSize=4, MarkerFaceColor = colorZ2,...
              Color = colorZ2)
        if mod(x/100,2) == 0
            marker = '^';
            quiver3(x,y,z2, 0,100,0,':', ...
                    LineWidth = 1,   ...
                    MaxHeadSize = 2, ...
                    Color=colorZ2)
        else
            quiver3(x,y+100,z2, 0,-100,0,':', ...
                    LineWidth = 1,   ...
                    MaxHeadSize = 2, ...
                    Color=colorZ2)
        end
    end
end

% -------------
fp1  = FlightPlan(Waypoint.empty);

wp1Y = Waypoint;
wp1Y.t   = 0;
wp1Y.pos = [  80  350   z0 ];
wp1Y.vel = [ 000  000  000 ];
fp1.SetWaypoint(wp1Y);

wp1Z = Waypoint;
wp1Z.t   = 20;
wp1Z.pos = [ 150  400   z1 ];
wp1Z.vel = [ 010  000  000 ];
fp1.SetWaypoint(wp1Z);

wp1A = Waypoint;
wp1A.t   = 40;
wp1A.pos = [ 350  400   z1 ];
wp1A.vel = [ 010  000  000 ];
fp1.SetWaypoint(wp1A);

wp1B = Waypoint;
wp1B.t   = 50;
wp1B.pos = [ 400  450   z2 ];
wp1B.vel = [ 000  010  000 ];
fp1.SetWaypoint(wp1B);

wp1C = Waypoint;
wp1C.t   = 80;
wp1C.pos = [ 400  750   z2 ];
wp1C.vel = [ 000  010  000 ];
fp1.SetWaypoint(wp1C);

wp1D = Waypoint;
wp1D.t   = 90;
wp1D.pos = [ 450  800   z1 ];
wp1D.vel = [ 010  000  000 ];
fp1.SetWaypoint(wp1D);

wp1E = Waypoint;
wp1E.t   = 120;
wp1E.pos = [ 750  800   z1 ];
wp1E.vel = [ 010  000  000 ];
fp1.SetWaypoint(wp1E);

wp1F = Waypoint;
wp1F.t   = 140;
wp1F.pos = [ 860  780   z0 ];
wp1F.vel = [ 000  000  000 ];
fp1.SetWaypoint(wp1F);

fp1.SetJLS()
tr = fp1.Trace(1);
tr = tr(1:end-1,:);

% pos
figure(1)
hold on

colorFP1 = [0.4660 0.6740 0.1880];
plot3(tr(:,2),tr(:,3),tr(:,4), ...
    '-', ...
    Color = colorFP1,...
    LineWidth = 3 );

plot3(tr(1,2),tr(1,3),tr(1,4), ...
    'o', ...
    MarkerSize = 10, ...
    MarkerFaceColor = colorFP1, ...
    MarkerEdgeColor = colorFP1);

plot3(tr(end,2),tr(end,3),tr(end,4), ...
    'o', ...
    MarkerSize = 10, ...
    MarkerFaceColor = colorFP1, ...
    MarkerEdgeColor = colorFP1);


% wp1A.PlotVel;
%wp1B.PlotVel;


%%------------------------------
% vel

fig2 = figure(2);
clf
grid on

% fig2.Position(3:4) = [400 300];
xlabel('Time (s)')
ylabel('Velocity (m/s)')
axis([0 200 0 12])


figure(2)
hold on
plot(tr(:,1),sqrt(tr(:,5).^2 + tr(:,6).^2 + tr(:,7).^2), ...
    '-', ...
    LineWidth = 1.5,...
    Color=colorFP1)



%%-----------------------------------------

% -------------
fp2  = FlightPlan(Waypoint.empty);

wp2Y = Waypoint;
wp2Y.t   = 9;
wp2Y.pos = [ 600  400   z0 ];
wp2Y.vel = [ 000  000  000 ];
fp2.SetWaypoint(wp2Y);

wp2Z = Waypoint;
wp2Z.t   = 30;
wp2Z.pos = [ 550  500   z1 ];
wp2Z.vel = [ -10  000  000 ];
fp2.SetWaypoint(wp2Z);

% wp2A = Waypoint;
% wp2A.t   = 40;
% wp2A.pos = [ 700  350   z2 ];
% wp2A.vel = [ 000  -10  000 ];
% fp2.SetWaypoint(wp2A);

% wp2B = Waypoint;
% wp2B.t   = 50;
% wp2B.pos = [ 650  300   z1 ];
% wp2B.vel = [ -10  000  000 ];
% fp2.SetWaypoint(wp2B);

wp2C = Waypoint;
wp2C.t   = 60;
wp2C.pos = [ 250  500   z1 ];
wp2C.vel = [ -10  000  000 ];
fp2.SetWaypoint(wp2C);

wp2D = Waypoint;
wp2D.t   = 70;
wp2D.pos = [ 200  550   z2 ];
wp2D.vel = [ 000  010  000 ];
fp2.SetWaypoint(wp2D);

wp2E = Waypoint;
wp2E.t   = 100;
wp2E.pos = [ 200  850   z2 ];
wp2E.vel = [ 000  010  000 ];
fp2.SetWaypoint(wp2E);

wp2F = Waypoint;
wp2F.t   = 110;
wp2F.pos = [ 150  900   z1 ];
wp2F.vel = [ -10  000  000 ];
fp2.SetWaypoint(wp2F);

wp2G = Waypoint;
wp2G.t   = 125;
wp2G.pos = [ 100  900   z0 ];
wp2G.vel = [ 000  000  000 ];
fp2.SetWaypoint(wp2G);



fp2.SetJLS()
tr = fp2.Trace(1);
tr = tr(1:end-1,:);

% pos
figure(1)
hold on

colorFP2 = [0.3010 0.7450 0.9330];
plot3(tr(:,2),tr(:,3),tr(:,4), ...
    '-', ...
    Color = colorFP2,...
    LineWidth = 3 );

plot3(tr(1,2),tr(1,3),tr(1,4), ...
    'o', ...
    MarkerSize = 10, ...
    MarkerFaceColor = colorFP2, ...
    MarkerEdgeColor = colorFP2);

plot3(tr(end,2),tr(end,3),tr(end,4), ...
    'o', ...
    MarkerSize = 10, ...
    MarkerFaceColor = colorFP2, ...
    MarkerEdgeColor = colorFP2);


%%------------------------------
% vel

figure(2)
hold on
plot(tr(:,1),sqrt(tr(:,5).^2 + tr(:,6).^2 + tr(:,7).^2), ...
    '-', ...
    LineWidth = 1.5, ...
    Color=colorFP2 )



