function [dis, ang] = Compute_RangeFinder(robot, laser, mapa, doprint)



% -----------------------------------------
% Check nargin
if nargin==3
    doprint=false;
end



% -----------------------------------------
% Define angles of the measurements
ang = linspace(-laser.angle,laser.angle,laser.nbeams);
%angles = angles(1);

% -----------------------------------------
% Get laser measurements
dis = zeros(1,length(ang));
for i=1:length(ang)
    beam = [robot.pose(1), robot.pose(2), robot.pose(3)+ang(i)];
    dis(i) = rangefinder(beam, robot.radio, mapa(:,1), mapa(:,2), laser.maxdis);
end

% -----------------------------------------
% Flip measurements from left to right: -laser_angle a laser_angle
dis = fliplr(dis);

% Add gaussian-distributed noise to the laser's measurements
dis = dis + laser.noisesigma*randn(size(dis));
%LaserRobot=LaserRobot/max(LaserRobot);

% -----------------------------------------
% Set measured distance to zero in case that an error exists
if isnan(sum(dis))
    dis = zeros(size(dis));
end

% -----------------------------------------
% Print  
if doprint
    disp(ang*180/pi)
    disp(dis)
end


% *************************************************************************
% rangefinder: calculate the distence of each laser beam
% *************************************************************************
function dist = rangefinder(posn,radiobot,Xmapa,Ymapa,laser_dis)

%para "laser_dis" pasos
for i = radiobot:(laser_dis+radiobot)
    %Obtiene las coordenadas (x,y) del punto de prueba del laser
    x = posn(1)+ i*sin(posn(3));
    y = posn(2)- i*cos(posn(3));
    
    x=round(x);
    y=round(y);
    %Asegurar que el punto de prueba del laser estan dentro del mapa
    if x>=max(Xmapa)
        x = max(Xmapa);
    elseif x<=min(Xmapa)
        x = min(Xmapa);
    end
    %Asegurar que el punto de prueba del laser estan dentro del mapa
    if y>=max(Ymapa)
        y = max(Ymapa);
    elseif x<=min(Ymapa)
        y = min(Ymapa);
    end
    
    ind = find(Xmapa==x);
    yycom = Ymapa(ind);
    for ii=1:length(yycom)
        if yycom(ii)==y
            dist = i-radiobot;
            return
        end
    end

end

%Si no se encontro obstaculo, entregar el maximo rango del laser
dist = laser_dis;
return;