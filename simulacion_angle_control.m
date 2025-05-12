clear; clc; close all;

% Parámetro del modelo
h = 0.05;    % Distancia h (del modelo con matriz D)
l = 0.2;     % Longitud usada para visualizar dirección

% Estado inicial
q = [0; 0; 0];  % [x; y; theta]
theta_dot = 0;

% Tiempo de simulación
dt = 0.01;
Tf = 10;
t = 0;

% Objetivo
q_d = [1; 1];

% Controladores
K = 5;
phi = 1;
thetad_dot = 0;


% Bandera de alineación
alineado = false;

while t < Tf
    x = q(1); y = q(2); theta = q(3);

    % Ángulo deseado (hacia la meta)
    thetad = atan2(q_d(2) - y, q_d(1) - x);
    thetae = wrapToPi(theta - thetad);

    % CONTROL DE ORIENTACIÓN
    if ~alineado
        u = phi * (-K * thetae + thetad_dot);  % Aceleración angular
        theta_dot = theta_dot + u * dt;        % Integración

        if abs(thetae) < 0.01
            alineado = true;
            theta_dot = 0;
        end
    else
        theta_dot = 0;
    end

    % Movimiento lineal
    d = norm(q_d - q(1:2));  % Distancia a la meta

    if alineado
        % Matriz D y control (modelo h)
        D = [cos(theta), -h*sin(theta);
             sin(theta),  h*cos(theta)];

        e = q(1:2) - q_d;
        u = -D \ e;

        gx = [cos(theta), -h*sin(theta);
              sin(theta),  h*cos(theta);
              0,           1];

        q = q + dt * gx * u;
    else
        % Solo gira, no avanza
        q(3) = q(3) + dt * theta_dot;
    end

    % Visualización
    clf;
    plot(q_d(1), q_d(2), 'rx', 'MarkerSize', 10, 'LineWidth', 2); hold on;
    plot(q(1), q(2), 'bo', 'MarkerSize', 8, 'LineWidth', 2);

    % Dirección deseada (rojo) y actual (verde)
    plot([q(1), q(1) + l*cos(thetad)], [q(2), q(2) + l*sin(thetad)], 'r-', 'LineWidth', 2);
    plot([q(1), q(1) + l*cos(q(3))],   [q(2), q(2) + l*sin(q(3))],   'g-', 'LineWidth', 2);

    grid on;
    axis equal;
    axis([-2, 2, -2, 2]);
    title(sprintf("Tiempo: %.2f s", t));

    drawnow;
    pause(0.01);
    t = t + dt;
end
