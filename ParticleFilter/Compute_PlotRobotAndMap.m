function Compute_PlotRobotAndMap(robot, laser, mapa, PrintInfo)



% -----------------------------------------
% Check nargin
if nargin==3
    PrintInfo=false;
end



% -----------------------------------------
% Plot map
plot(mapa(:,1),mapa(:,2),'.k')
xlabel('x'), ylabel('y')
hold on;
axis([-80 80 -80 80])



% -----------------------------------------
% Plot a circle representing the robot
angs = 0:pi/100:2*pi;
x = robot.pose(1) + robot.radio*sin(angs);
y = robot.pose(2) + robot.radio*cos(angs);
plot(x,y,'r');
plot(robot.pose(1),robot.pose(2),'r')
% Compute_DrawAng(robot.pose, robot.radio+laser.maxdis)



% -----------------------------------------
% Plot laser's beams
angles = linspace(-laser.angle, laser.angle, laser.nbeams); % Dibujar todos los beams
%angles = [-laser.angle, laser.angle]; % Dibujar solo las dos externa beams mas externos
for i=1:length(angles)
    y = [robot.pose(1), robot.pose(1)+ (laser.maxdis+robot.radio)*sin(robot.pose(3)+angles(i))];
    x = [robot.pose(2), robot.pose(2)- (laser.maxdis+robot.radio)*cos(robot.pose(3)+angles(i))];
    plot(y,x,'r');
end



% -----------------------------------------
% Plot laser's arc
angs = (-laser.angle:pi/100:laser.angle)+robot.pose(3);
x = robot.pose(1) + (robot.radio+laser.maxdis)*sin(angs);
y = robot.pose(2) - (robot.radio+laser.maxdis)*cos(angs);
plot(x,y,'r')
% % Dibujar arcos del laser
% for i=1:laser_dis
%     x = robot.pose(1) + (robot.radio+i)*sin(angs);
%     y = robot.pose(2) - (robot.radio+i)*cos(angs);
%     plot(x,y,'r');
% end



% -----------------------------------------
% Print robot's pose and laser's measurements
if PrintInfo
    % Print robot's pose
    title( sprintf('Pose: (%.1f, %.1f, %.1f)', robot.pose(1), robot.pose(2), 180*robot.pose(3)/pi ) )

    % Print laser's measurements
    if laser.nbeams==7
        textx_pos = -75;
        text_ypos = +70;
        text(textx_pos,text_ypos,sprintf('Range finder: (%.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f)m',laser.rangefinder(1),laser.rangefinder(2),laser.rangefinder(3),laser.rangefinder(4),laser.rangefinder(5),laser.rangefinder(6),laser.rangefinder(7)),'Color','red','FontSize',10)
    end

end



% -----------------------------------------
% Set plot
axis on
axis square
drawnow
hold off