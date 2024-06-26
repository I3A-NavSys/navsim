clc
clear
run('../../../tools/NAVSIM_PATHS');


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
    wp.pos = way_data1(i,2:4);
    fp1.SetWaypoint(wp);
end

fp1.SetV0000;

% Display
fp1.PositionFigure("FP1: POSITION",1);
fp1.VelocityFigure("FP1: VELOCITY",1);


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
    wp.pos = way_data2(i,2:4);
    fp2.SetWaypoint(wp);
end

fp2.SetV0000;

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
    wp.pos = way_data3(i,2:4);
    fp3.SetWaypoint(wp);
end

fp3.SetV0000;

