%% Este script muestra las diferencias entre los 
%  modos de definición de planes de vuelo 
%  TP   (tiempo posición)
%  TPV  (tiempo posición velocidad)

clc; clear;         %Clean console and workspace

time_step = 0.1;


%-------------
%Create FP1

%             t   x  y  z  vx vy vz
way_data = [ 05  00 05 00  00 00 00
             10  00 05 00  00 00 00
             20  00 05 05  00 00 01
             30  10 05 10  00 00 -1 
             40  10 05 00  00 00 00 
             50  10 05 00  00 00 00 ];
    
fp1  = FlightPlan(1,Waypoint.empty);
for i = 1:size(way_data,1)
    wp = Waypoint();
    wp.t = way_data(i,1);
    wp.SetPosition(way_data(i,2:4));
    fp1.SetWaypoint(wp);
end

% Display
fp1.RouteFigure(time_step)
fp1.VelocityFigure(time_step)

  
%-------------
%Create FP2
for i = 1:size(way_data,1)
    waypoints(i) = Waypoint();
    waypoints(i).t = way_data(i,1);
    waypoints(i).SetPosition(way_data(i,2:4));
    waypoints(i).SetVelocity(way_data(i,5:7));
end
fp2  = FlightPlan(2,waypoints);
fp2.mode = "TPV";

% Display
fp2.RouteFigure(time_step)
fp2.VelocityFigure(time_step)


%-------------
% Relative distance between two flightplans
fp1.DistanceFigure(fp2,time_step)




