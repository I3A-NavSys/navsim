clc 

% -------------
%Create a Flight Plan for the drone 1

%              t      x        y        z    
way_data1 = [ 05   -190.00  -119.00  +048.05   
              10   -190.00  -119.00  +052.00   
              30   -190.00  -159.00  +052.00   
              50   -150.00  -159.00  +052.00   
              70   -150.00  -119.00  +052.00   
              90   -190.00  -119.00  +052.00   
              95   -190.00  -119.00  +048.05  
              97   -190.00  -119.00  +048.05  ];

fp1  = FlightPlan(Waypoint.empty);

for i = 1:size(way_data1,1)
    wp = Waypoint();
    wp.t = way_data1(i,1);
    wp.SetPosition(way_data1(i,2:4));
    fp1.SetWaypoint(wp);
end

% Display
fp1.PositionFigure(1,1);
fp1.VelocityFigure(1,1);

% -------------
%Create a Flight Plan for the drone 2

%              t      x        y        z    
way_data2 = [ 10   -152.00  -106.00  +049.05   
              15   -152.00  -106.00  +051.90   
              25   -190.00  -119.00  +051.90
              30   -190.00  -119.00  +051.90
              35   -190.00  -119.00  +048.10  ];

fp2  = FlightPlan(Waypoint.empty);
for i = 1:size(way_data2,1)
    wp = Waypoint();
    wp.t = way_data2(i,1);
    wp.SetPosition(way_data2(i,2:4));
    fp2.SetWaypoint(wp);
end

% -------------
%Create a Flight Plan for the drone 3

%              t      x        y        z    
way_data3 = [  5   +180.00  +033.00  +050.00
              10   +180.00  +033.00  +052.00   
              20   +130.00  +000.00  +052.00
              60   -190.00  -119.00  +052.00
              70   -190.00  -119.00  +052.00
              75   -190.00  -119.00  +048.30  ];

fp3  = FlightPlan(Waypoint.empty);
for i = 1:size(way_data3,1)
    wp = Waypoint();
    wp.t = way_data3(i,1);
    wp.SetPosition(way_data3(i,2:4));
    fp3.SetWaypoint(wp);
end



% -------------
%Comunicate Flight Plans
operator.ResetSim;
operator.SendFlightPlan('UAV01',fp1);
operator.SendFlightPlan('UAV02',fp2);
operator.SendFlightPlan('UAV03',fp3);



%%
operator.WaitTime(max([fp1.FinishTime fp2.FinishTime fp3.FinishTime]));
% operator.RemoveUAV('UAV01');
% operator.RemoveUAV('UAV02');
% operator.RemoveUAV('UAV03');
operator.PauseSim;

monitor.PositionFigure('UAV01',fp1);
monitor.VelocityFigure('UAV01',fp1);
[medE,maxE,t] = monitor.PathFollowingError('UAV01',fp1);

monitor.PositionFigure('UAV02',fp2);
monitor.VelocityFigure('UAV02',fp2);

monitor.PositionFigure('UAV03',fp3);
monitor.VelocityFigure('UAV03',fp3);




