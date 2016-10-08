%% section1

close all;
clearvars;
curve = CubicSpiralTrajectory([-3.0 1.55 3.0],1001);
pose = curve.getFinalPose();
fprintf('x:%f y:%f t:%f\n',pose(1),pose(2),pose(3));

%% section2
close all;
clearvars;
curve = CubicSpiralTrajectory();
curve.makeLookupTable(80);


% curve = CubicSpiralTrajectory([-3.0 1.55 3.0],10001);
% pose = curve.getFinalPose();
% plot(pose(1),pose(2));
% fprintf('x:%f y:%f t:%f\n',pose(1),pose(2),pose(3));

% curve = CubicSpiralTrajectory([-3.0 1.55 3.0],100001);
% pose = curve.getFinalPose();
% plot(pose(1),pose(2));
% fprintf('x:%f y:%f t:%f\n',pose(1),pose(2),pose(3));

%% Challenge Task
clearvars -except robot;
close all;
format long g;
if(~exist('robot', 'var'))
    robot = raspbot();
    pause(2);
end
robot.encoders.NewMessageFcn=@encoderEventListener;
clearvars -except robot;
pause(1);
if(~exist('cubicSpirals.mat', 'file'))
    CubicSpiralTrajectory.makeLookupTable(80);
end
 
 curve = CubicSpiralTrajectory.planTrajectory(.25, .25, 0.0, 1);
 curve.planVelocities(.25);
 %traj_model = RobotTrajectory(traj_del, traj_ref); %velocities go here somehow
 
 trajFollower = TrajectoryFollowerC(curve, 1);
 trajFollower.executeTrajectory(robot, .2);
 
 robot.encoders.NewMessageFcn=[];