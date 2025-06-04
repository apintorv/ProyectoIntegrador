%% ROBOT



%% INITIALIZE
clear
close all
clc



%% ROBOT

% -----------------------------------------
% Robot's radio and pose
robot.radio = 4;                  % radio
robot.pose = [-10, 0, 4*pi/4];    % (x,y,theta)

% -----------------------------------------
% Laser
laser.maxdis     = 30;   % Maximum measured distance
laser.angle      = pi/4; % Angle (field of view)
laser.nbeams     = 7;    % Number of beams (only odd number)
laser.noisesigma = 0.1;  % Standard deviation of the noise in the laser

% -----------------------------------------
% Plot robot
figure(1), clf, %set(gcf, 'Position', [141.6667  421.6667  560.0000  421.3333])
Compute_PlotRobotAndMap(robot, laser, [NaN, NaN])
title('Robot')

% -----------------------------------------
% Remove garbage
clear ans robot.pose



% TASK: understand the robot, change parameters and see the new results



%% MAP

% -----------------------------------------
% Load map
load('mapa1.mat')

% -----------------------------------------
% Plot map
figure(2), %set(gcf, 'Position', [141.6667  421.6667  560.0000  421.3333])
plot(mapa(:,1),mapa(:,2),'.k')
xlabel('x (m)'), ylabel('y (m)')
title('Map')



% TASK: understand the maps, change the map and see the new plot



%% PLOT MAP AND ROBOT: FIXED POSITION

% -----------------------------------------
% Define a pose for the robot
robot.pose = [-10, 0, 4*pi/4];    % (x,y,theta)

% -----------------------------------------
% Plot robot and map
figure(3)
Compute_PlotRobotAndMap(robot, laser, mapa)
title('Map and Robot')



% TASK: understand the robot's pose, change the pose and see the new
% results



%% MOVE ROBOT IN "x"

% -----------------------------------------
% Define x-positions for the robot
X = -50:1:50;

% -----------------------------------------
% For each x
for i=1:1:length(X)

    % Get robot curent x-position
    x = X(i);

    % Define current pose for the robot (x,y,theta)
    robot.pose = [x, -4.7, 2*pi/4];

    % Plot
    figure(4)
    Compute_PlotRobotAndMap(robot, laser, mapa)
    title( sprintf('(x, y, theta) = (%.1f, %.1f, %.1f)', robot.pose(1), robot.pose(2), 180*robot.pose(3)/pi ) )

    %pause(0.1)

end

% -----------------------------------------
% Remove garbage
clear ans i X x



% TASK: understand the robot's movements, change the initial pose and see
% the new results. What is the problem during movement?



%% MOVE ROBOT IN "y"

% -----------------------------------------
% Define y-positions for the robot
% X = -50:1:50;

% TASK: create the code to move the robot

% TASK: understand the robot's movements, change the initial pose and see
% the new results. What is the problem during movement?

%% ROTATE ROBOT

% -----------------------------------------
% Define orientations for the robot
% THETA = 0:pi/5:20*pi;



% TASK: create the code to move the robot

% TASK: understand the robot's movements, change the initial pose and see
% the new results. What is the problem during movement?



%% OBTAIN LASER'S MEASUREMENTS

% -----------------------------------------
% Define a position for the robot (x,y,theta)
robot.pose = [1.3, -11.3, 2.5*pi/6];
% robot.pose = [-0.8, 25.3, 9*pi/6];

% -----------------------------------------
% Get laser's measurements
laser.rangefinder = Compute_RangeFinder(robot, laser, mapa, false);

% -----------------------------------------
% Plot map, robot, and laser measurements
figure(7)
Compute_PlotRobotAndMap(robot, laser, mapa, true)



% TASK: change pose and check the observations. Does it have sense?



%% MOVE ROBOT IN "x" AND GET LASER MEASUREMENTS

% -----------------------------------------
% Define x-positions for the robot
X = -50:1:50;

% -----------------------------------------
% For each x
for i=1:1:length(X)

    % Get robot curent x-position
    x = X(i);

    % Define current pose for the robot (x,y,theta)
    robot.pose = [x, -4.7, 2*pi/4];

    % Get laser's measurements
    laser.rangefinder = Compute_RangeFinder(robot, laser, mapa, false);

    % Plot
    figure(8)
    Compute_PlotRobotAndMap(robot, laser, mapa, true)

    %pause(0.1)

end

% -----------------------------------------
% Remove garbage
clear ans i X x



% TASK: understand the robot's movement and measurements



%% MOVE THE ROBOT AND GET ODOMETRY

% -----------------------------------------
% Clear screen
clc

% -----------------------------------------
% Define driving information
drive.wdia          = 7;   % Distance between robot's wheels
drive.dt            = .5;  % Time step 
% drive.vLmax         = 5;   % Maximum left wheel velocity
% drive.vRmax         = 5;   % Maximum right wheel velocity
% drive.Li            = 10;  % Minimum laser distance to decide to turn

% drive.robotstatus   = 'No blocked';
% drive.turningsense  = 'forward';
drive.vL            = 10; % Left wheel velocity
drive.vR            = 10; % Right wheel velocity

% drive.robotstatus   = 'Blocked in the right';
% drive.turningsense  = 'left';
% drive.vL            = -5; % Left wheel velocity
% drive.vR            = +5; % Right wheel velocity

% drive.robotstatus   = 'Blocked in the left';
% drive.turningsense  = 'right';
% drive.vL            = +5; % Left wheel velocity
% drive.vR            = -5; % Right wheel velocity

% drive.robotstatus   = 'Blocked in the ?';
% drive.turningsense  = 'move in all dimensions';
% drive.vL            = +10; % Left wheel velocity
% drive.vR            = +10; % Right wheel velocity

% -----------------------------------------
% Define robot's pose
robot.pose = [0, -40, 4*pi/4];
fprintf('Act pose: (%.1f, %.1f, %.1f) \n', robot.pose(1), robot.pose(2), 180*robot.pose(3)/pi )

% Get laser's measurements
laser.rangefinder = Compute_RangeFinder(robot, laser, mapa);

% Plot map and robot in the current position
figure(9), clf
Compute_PlotRobotAndMap(robot, laser, mapa, false)



pause



% -----------------------------------------
% New position
[drive.odometry,drive.posenew] = Compute_Drive(robot.pose, drive.wdia, drive.vL, drive.vR, drive.dt);
robot.pose = robot.pose + drive.odometry;

% Get laser's measurements
laser.rangefinder = Compute_RangeFinder(robot, laser, mapa);

% Plot map and robot in the new position
hold on
Compute_PlotRobotAndMap(robot, laser, mapa, false)

% Print new pose
fprintf('Odometry: (%.1f, %.1f, %.1f) \n', drive.odometry(1), drive.odometry(2), 180*drive.odometry(3)/pi )
fprintf('New pose: (%.1f, %.1f, %.1f) \n', drive.posenew(1), drive.posenew(2), 180*drive.posenew(3)/pi )



% TASK: understand the robot's movement and the odometry


