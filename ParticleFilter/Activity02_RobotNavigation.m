%% MOVING ROBOT



%% INITIALIZE
clear
close all
clc



%% DEFINE ROBOT AND LASER

% -----------------------------------------
% Robot's radio and pose
robot.radio = 4;                  % radio
robot.pose  = [-10, 0, 4*pi/4];   % (x,y,theta)


% -----------------------------------------
% Laser
laser.maxdis     = 30;   % Maximum measured distance
laser.angle      = pi/4; % Angle (field of view)
laser.nbeams     = 5;    % Number of beams (only odd number)
laser.noisesigma = 0.1;  % Noiseless because we want to move



%% LOAD MAP

% -----------------------------------------
% Load map
load('mapa2.mat')



%% FREELY MOVEMENT OF THE ROBOT IN THE MAP

clc

% -----------------------------------------
% Define initials robot's pose
% robot.pose = [0, -55, 4*pi/4];
% robot.pose = [0, 55, 4*pi/4];
% robot.pose = [-8.7, -31.9, -212.9*pi/180];
% robot.pose = [50.0, 50.0, 6*pi/4];
robot.pose = [-20,0,1];

% -----------------------------------------
% Define number of steps
Nsteps = 1000;

% -----------------------------------------
% Initialize path
path = nan(Nsteps,3);

% -----------------------------------------
% For each time step:
for i=1:1:Nsteps
    fprintf('Step %d of %d \n', i, Nsteps )


    % -----------------------------------------
    % Save current pose
    path(i,:) = robot.pose;

    % -----------------------------------------
    % Get laser's measurements
    laser.rangefinder = Compute_RangeFinder(robot, laser, mapa, false);

    % -----------------------------------------
    % Plot map, robot, and laser measurements
    figure(1), clf
    Compute_PlotRobotAndMap(robot, laser, mapa, true)

    % -----------------------------------------
    % Compute new position
    [odometry, ~] = Compute_MoveRobot(robot, mapa, false, false);
    robot.pose = robot.pose + odometry;
    robot.pose(3) = AjustaAngulo(robot.pose(3));

    %pause()

end % for i=1:1:Nsteps

hold on
plot(path(:,1),path(:,2),'b')

% -----------------------------------------
% Remove garbage
clear ans i odometry posenew


% TASK: understand the robot's nagivation. How does it take decisions to
% move/navigate? 


