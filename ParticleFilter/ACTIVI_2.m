%% ROBOT LOCALIZATION USING PARTICLE FILTER



%% INITIALIZE
clear
close all
clc



%% DEFINE ROBOT AND LASER

% -----------------------------------------
% Robot's radio and pose
robot.radio = 4;                  % radio
robot.pose = [-10, 0, 4*pi/4];    % (x,y,theta)
% robot.pose = [0, -55, 4*pi/4];
% robot.pose = [0, 55, 4*pi/4];
% robot.pose = [-8.7, -31.9, -212.9*pi/180];
% robot.pose = [50.0, 50.0, 6*pi/4];

% -----------------------------------------
% Initialize odometria
robot.odometria = [0 0 0];

% -----------------------------------------
% Laser
laser.maxdis     = 30;   % Maximum measured distance
laser.angle      = pi/4; % Angle (field of view)
laser.nbeams     = 7;    % Number of beams (only odd number)
laser.noisesigma = 0.1;  % Noiseless because we want to move

% -----------------------------------------
% Remove garbage
clear ans robotpos



%% LOAD MAP

% -----------------------------------------
% Define filename
filename = 'mapa2';

% -----------------------------------------
% Load map
load(filename)



%% PLOT

% Compute observations
laser.rangefinder  = Compute_RangeFinder(robot, laser, mapa, false);

% Plot map, robot, laser
figure(1), clf
Compute_PlotRobotAndMap(robot, laser, mapa, true)



%% LOCALIZATION PARTICLE FILTER

% Define number of particulas
particles.NP = 100; % (Pregunta 3 de la actividad)

% Inicialize particulas
particles.X  = (max(mapa(:,1))-1)*2*(0.5-rand(particles.NP,1));
particles.Y  = (max(mapa(:,1))-1)*2*(0.5-rand(particles.NP,1));
particles.T  = (2*pi*rand(particles.NP,1))-pi;

% Plot map, robot, laser and particles (also plot the direction)
figure(1), clf
Compute_PlotRobotAndMap(robot, laser, mapa, true)
hold on
plot(particles.X,particles.Y,'.b')

% Weight vector
particles.W  = zeros(particles.NP,1);

% Plot particle-distribution of x, y, and z, and the real robot pose x, y
% theta (Pregunta 1 de la actividad) 

%% FOR EACH TIME STEP

% -----------------------------------------
% Define number of steps
Nsteps = 1000; % (Pregunta 4 de la actividad)

% -----------------------------------------
% For step
for i=1:1:Nsteps
    fprintf('Time steps: i = %i / %i  \r',i,Nsteps);

    % -----------------------------------------
    % 1) Get robot laser measurements
    laser.rangefinder  = Compute_RangeFinder(robot, laser, mapa, false);

    % -----------------------------------------
    % 2) Plot map, robot, laser measurements, and particles
    figure(1), clf
    Compute_PlotRobotAndMap(robot, laser, mapa, true)
    hold on
    plot(particles.X,particles.Y,'.b')
    alpha(0.01)
    drawnow
    %pause(0.25)
    
    % % -----------------------------------------
    % % 3) Localization algorithm based on particles filter
    particles = Compute_LocalizationParticleFilter(robot,laser,mapa,particles); % (Pregunta 5 y 6 de la actividad)
    
    % % -----------------------------------------
    % % 4) Move robot and compute odometry
    [robot.odometria,~] = Compute_MoveRobot(robot, mapa, false, false);
    robot.pose = robot.pose + robot.odometria;

    % % -----------------------------------------
    % % 5) Plot particle-distribution of x and y, and the real position x
    % and y (Pregunta 2 de la actividad)
    figure(2), clf
    subplot(1,2,1)
    histogram(particles.X, 20)
    hold on
    xline(robot.pose(1), 'r', 'LineWidth', 2)
    title('Histograma de X')
    xlabel('x')
    ylabel('Frecuencia')
    legend('Partículas', 'Robot real')
    
    subplot(1,2,2)
    histogram(particles.Y, 20)
    hold on
    xline(robot.pose(2), 'r', 'LineWidth', 2)
    title('Histograma de Y')
    xlabel('y')
    ylabel('Frecuencia')
    legend('Partículas', 'Robot real')

end

% -----------------------------------------
% Remove garbage
clear ans i odometria robotpos

%% Actividad:
%
% 1) Graficar el histograma de x, y, y theta de las particulas inciales. 
% Que tipo de PDF es esta? Gaussiana? Uniforme? otra?
% Es uniforme porque todas las particulas tienen la misma probabilidad
% 
% 2) En una figura, graficar:
%   a) Histrogram of "x" (particles) and the real value of "x"
%   b) Histrogram of "y" (particles) and the real value of "y"
% figure(2), clf
% subplot(1,2,1)
% histogram(particles.X, 20)
% hold on
% xline(robot.pose(1), 'r', 'LineWidth', 2)
% title('Histograma de X')
% xlabel('x')
% ylabel('Frecuencia')
% legend('Partículas', 'Robot real')
% 
% subplot(1,2,2)
% histogram(particles.Y, 20)
% hold on
% xline(robot.pose(2), 'r', 'LineWidth', 2)
% title('Histograma de Y')
% xlabel('y')
% ylabel('Frecuencia')
% legend('Partículas', 'Robot real')
% 
% 3) Cual es el efecto de usar menos o mas particulas?
%  Que la covarianza aumenta o disminuye, o sea, la probabilidad de que
%  este en ese lugar es mas o menos confiable
% 
% 4) Cual es el efecto de contar con mas o menos time-steps?
% Con un step muy pequeño, no le da tiempo de "encontrar" o calcular cual
% es la ubicacion precisa del robot
% 
% 5) Escribir el algoritmo y explicar el funcionamiento del filtro de
% particulas
% - Crear particulas aleatorias a lo largo del mapa
% - Se mueve el robot y se aplica la odometria a cada particula
% - Se compara la medicion de los lasers con las predichas por cada
% particula
% - Se calcula el peso (probabilidad) de cada particula segun su medicion
% - Se eliminan las particulas de bajo peso y se agregan cerca de las mayor
% peso
% - Se repite el proceso

% 6) Escribir y explicar la ecuacion para calcular el peso de cada
% particula
% particles.W(ipar) = exp(-0.5 *(laser.rangefinder-particlesiclelaser.rangefinder)*(laser.rangefinder-particlesiclelaser.rangefinder)');
% Cuanto mas se parecen a las medidas simuladas a las reales, mayor es su peso. 

% 7) Como se puede usar este algoritmo como parte de solucion del reto?
% Para evitar obstaculos, localizacion del robot y planeacion de
% trayectoria
