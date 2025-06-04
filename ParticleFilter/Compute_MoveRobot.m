function [odometry, posenew] = Compute_MoveRobot(robot, mapa, doplot, doprint)



%% INITIALIZATIONS

% -----------------------------------------
% Laser to determine how to move
laser.maxdis     = 30;   % Maximum measured distance
laser.angle      = pi/4; % Angle (field of view)
laser.nbeams     = 3;    % Number of beams (only odd number)
laser.noisesigma = 0;    % Noiseless (we only want to move while avoiding obstacles)

% -----------------------------------------
% Define driving information
drive.wdia          = 7;   % Distance between robot's wheels
drive.dt            = .5;  % Time step
drive.vLmax         = 5;   % Maximum left wheel velocity
drive.vRmax         = 5;   % Maximum right wheel velocity
drive.vL            = [];  % Left wheel velocity
drive.vR            = [];  % Right wheel velocity
drive.Li            = laser.maxdis/3;  % Minimum laser distance to decide to turn


%% GET INFO OF THE CURRENT POSITION

% -----------------------------------------
% Get laser's measurements
[laser.rangefinder,laser.rangeangles] = Compute_RangeFinder(robot, laser, mapa, false);

rangeL = laser.rangefinder(1);
rangeC = laser.rangefinder(2);
rangeR = laser.rangefinder(3);

% -----------------------------------------
% Plot map, robot, and laser
if doplot
    Compute_PlotRobotAndMap(robot, laser, mapa, false)
end

% -----------------------------------------
% Print
if doprint
    fprintf('********************************************************* \n')

    fprintf('Laser rangeangles: (%.1f, %.1f, %.1f) \n', 180*laser.rangeangles(1)/pi, 180*laser.rangeangles(2)/pi, 180*laser.rangeangles(3)/pi )
    fprintf('Laser rangefinder: (%.1f, %.1f, %.1f) \n', rangeL, rangeC, rangeR )

    fprintf('\n')
    fprintf('Act pose: (%.1f, %.1f, %.1f) \n', robot.pose(1), robot.pose(2), 180*robot.pose(3)/pi )
end



%% DETERMINE VELOCITIES (LEFT AND RIGHT WHEELS)
%
%              True table
% --------------------------------------
%   L C R  |  blocked in |  action
% --------------------------------------
%   0 0 0  |             | move forward
%   0 0 1  |  R          |
%   0 1 0  |  C          |
%   0 1 1  |  R & C      |
%   1 0 0  |  L          |
%   1 0 1  |  L & R      | 
%   1 1 0  |  L & C      | turn right
%   1 1 1  |  L & R & C  | turn towards the side with the larger rangefinder distance or randomly if equal distance in the sided

L = rangeL<=drive.Li; % True: bloqued in the left
C = rangeC<=drive.Li/2; % True: bloqued in the center
R = rangeR<=drive.Li; % True: bloqued in the right

Vrandmax = 1.5;
Vrandmin = 0.5;
Vrand    = (Vrandmax-Vrandmin).*rand + Vrandmin;


% -----------------------------------------
% Determine velocities of the left and right wheels

if (~L && ~C && ~R) % (0,0,0)
    % If not blocked anywhere (left, center, right)
    drive.robotstatus   = 'No blocked';
    drive.turningsense  = 'forward';
    drive.vL = drive.vLmax;
    drive.vR = drive.vRmax;
    

elseif (~L && ~C && R) % (0,0,1) 
    % If blocked in the right
    drive.robotstatus   = 'Blocked in right';    
    %drive.turningsense  = 'left';
    %drive.vL = -1;
    %drive.vR = +1;
    if rand>0.5
        drive.turningsense  = 'left';
        %drive.vL = -1;
        %drive.vR = +1;
        drive.vL = - Vrand;
        drive.vR = + Vrand;
    else
        drive.turningsense  = 'forward';
        %drive.vL = +1;
        %drive.vR = +1;
        drive.vL = + Vrand;
        drive.vR = + Vrand;     
    end
    
elseif (~L && C && ~R) % (0,1,0)
    % If blocked in the center
    drive.robotstatus   = 'Blocked in center';

    % Turn towards the side with the larger rangefinder distance
    if (rangeL >= rangeR+2)
        % The left distance is larger: turn  left
        drive.turningsense  = 'left';
        %drive.vL = -1;
        %drive.vR = +1;
        drive.vL = - Vrand;
        drive.vR = + Vrand;
    elseif (rangeR >= rangeL+2)
        % The right distance is larger: turn  right
        drive.turningsense  = 'right';
        %drive.vL = +1;
        %drive.vR = -1;
        drive.vL = + Vrand;
        drive.vR = - Vrand;
    else
        % Equal distance in the left and in the right: turn randomly
        drive.vL = Vrand*sign(.5-rand);
        drive.vR = -drive.vL;
        if drive.vL<0, drive.turningsense  = 'randomly towards the left'; end
        if drive.vR<0, drive.turningsense  = 'randomly towards the right'; end
    end % if (rangeL >= rangeR+2)


elseif (~L && C && R) % (0,1,1)
    % If blocked in the right and in the center    
    drive.robotstatus   = 'Blocked in right and in the center';
    drive.turningsense  = 'left';
    %drive.vL = -1;
    %drive.vR = +1;
    drive.vL = - Vrand;
    drive.vR = - drive.vL;


elseif (L && ~C && ~R) % (1,0,0)
    % If blocked in the left
    drive.robotstatus   = 'Blocked in left';
    % drive.turningsense  = 'right';
    % drive.vL = + ( (1.0-0.8).*rand + 0.8 );
    % drive.vR = - drive.vL;    
    if rand>0.5
        drive.turningsense  = 'right';
        %drive.vL = +1;
        %drive.vR = -1;
        drive.vL = + Vrand;
        drive.vR = - Vrand;
    else
        drive.turningsense  = 'forward';
        %drive.vL = +1;
        %drive.vR = +1;
        drive.vL = + Vrand;
        drive.vR = + Vrand;        
    end


elseif (L && ~C && R) % (1,0,1)
    % If blocked in the left and in the right
    drive.robotstatus   = 'Blocked in left and right';
    drive.turningsense  = 'forward';
    %drive.vL = +1;
    %drive.vR = +1;
    drive.vL = + Vrand;
    drive.vR = + drive.vL;


elseif (L && C && ~R) % (1,1,0)
    % -----------------------------------------
    % If blocked in the left and in the center
    drive.robotstatus   = 'Blocked in left and in the center';
    drive.turningsense  = 'right';
    %drive.vL = +1;
    %drive.vR = -1;
    drive.vL = + Vrand;
    drive.vR = - Vrand;


elseif (L && C && R) % (1,1,1)
    % -----------------------------------------
    % If blocked in both sides and in the center
    drive.robotstatus   = 'Blocked in left, right and center';

    % Turn towards the side with the larger rangefinder distance
    if (rangeL >= rangeR+2)
        % The left distance is larger: turn  left
        drive.turningsense  = 'left';
        drive.vL = -1;
        drive.vR = +1;
        drive.vL = -Vrand;
        drive.vR = +Vrand;
    elseif (rangeR >= rangeL+2)
        % The right distance is larger: turn  right
        drive.turningsense  = 'right';
        drive.vL = +1;
        drive.vR = -1;
        drive.vL = +Vrand;
        drive.vR = -Vrand;
    else
        % Equal distance in the left and in the right: turn randomly
        drive.vL = Vrand*sign(.5-rand);
        drive.vR = -drive.vL;
        if drive.vL<0, drive.turningsense  = 'randomly towards the left'; end
        if drive.vR<0, drive.turningsense  = 'randomly towards the right'; end
    end % if (rangeL >= rangeR+2)

end % if (~L && ~C && ~R) % (0,0,0)



%% MOVE THE ROBOT AND GET NEW POSITION AND ODOMETRY

% -----------------------------------------
% Verificar de que no vaya de pa'tras
if drive.vL<0 && drive.vR<0
    error('No se puede moverse hacia atras')
end


% -----------------------------------------
% Get new position
[drive.odometry,drive.posenew] = Compute_Drive(robot.pose, drive.wdia, drive.vL, drive.vR, drive.dt);

% % Plot robot in the new position
% if doplot
%     robot.pose = robot.pose + drive.odometry;
%     %pause(0.5)
%     hold on
%     Compute_PlotRobotAndMap(robot, laser, mapa, false)
% end


% -----------------------------------------
% Print new pose
if doprint
    %fprintf('vL: %.1f \n', drive.vL )
    %fprintf('vR: %.1f \n', drive.vR )
    fprintf('Odometry: (%.1f, %.1f, %.1f) \n', drive.odometry(1), drive.odometry(2), 180*drive.odometry(3)/pi )
    fprintf('New pose: (%.1f, %.1f, %.1f) \n', drive.posenew(1), drive.posenew(2), 180*drive.posenew(3)/pi )
    fprintf('Status:  %s \n',drive.robotstatus)
    fprintf('Turning: %s \n',drive.turningsense)
end


% -----------------------------------------
% Save output variables
odometry = drive.odometry;
posenew  = drive.posenew;


% -----------------------------------------
% Decide velocity of the wheels
% function  vL, Vr, turningsense = DecideWheelsVel()
