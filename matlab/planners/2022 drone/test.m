clear;clc

traj = Path5D;
p1 = Pose5D;

traj.setWaypoint(00,000,000,000,0);
traj.setWaypoint(05,000,000,000,0);
traj.setWaypoint(10,000,000,010,4);
traj.setWaypoint(20,040,000,010,4);
traj.setWaypoint(30,040,060,010,8);
traj.setWaypoint(40,000,060,010,0);
traj.setWaypoint(45,000,060,010,0);
traj.setWaypoint(50,000,060,000,0);
traj.setWaypoint(51,000,060,000,0);

step = 1;
traj.showPoses4D("trajectory4D",step);
xlim([0 60])
ylim([0 60])
zlim([0 60])

traj.showVels4D("velocity4D",step);
ylim([0 9])



%%

traj.showPoses5D("trajectory5D",step);
xlim([0 60])
ylim([0 60])
zlim([0 60])

traj.showVels5D("velocity5D",step);
ylim([0 9])
