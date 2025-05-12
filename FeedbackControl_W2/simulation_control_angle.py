import numpy as np
import matplotlib.pyplot as plt

# Parámetros
h = 0.05        # parámetro de la cinemática
l = 0.2         # largo para visualización
dt = 0.01       # paso de integración
Tf = 10         # tiempo total de simulación

# Estado inicial
q = np.array([0.0, 0.0, 0.0])  # [x, y, theta]
theta_dot = 0.0

# Objetivo
q_d = np.array([-1.0, 1.0])  # [x_d, y_d]

# Control
K = 5
phi = 1
thetad_dot = 0.0
umbral_angulo = 0.01  # umbral para considerar alineado

# Bandera de alineación
alineado = False

# Función wrapToPi
def wrapToPi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

# Visualización interactiva
plt.ion()
fig, ax = plt.subplots()

t = 0.0
while t < Tf:
    x, y, theta = q
    thetad = np.arctan2(q_d[1] - y, q_d[0] - x)  # Dirección hacia el objetivo
    thetae = wrapToPi(theta - thetad)  # Error angular

    # Control de orientación
    if not alineado:
        u = phi * (-K * thetae + thetad_dot)
        theta_dot += u * dt
        if abs(thetae) < umbral_angulo:
            alineado = True
            theta_dot = 0.0
    else:
        theta_dot = 0.0

    # Movimiento lineal solo si está alineado
    if alineado:
        # Matriz D de cinemática
        D = np.array([
            [np.cos(theta), -h * np.sin(theta)],
            [np.sin(theta),  h * np.cos(theta)]
        ])

        # Error de posición
        e = q[:2] - q_d

        # Control de movimiento
        u = -np.linalg.solve(D, e)

        # Cinemática completa
        gx = np.array([
            [np.cos(theta), -h * np.sin(theta)],
            [np.sin(theta),  h * np.cos(theta)],
            [0.0,            1.0]
        ])
        q = q + dt * gx @ u
    else:
        # Solo ajusta theta
        q[2] = q[2] + theta_dot * dt

    # Visualización
    ax.clear()
    ax.plot(q_d[0], q_d[1], 'ro', label='Objetivo')
    ax.plot([x, x + l * np.cos(thetad)], [y, y + l * np.sin(thetad)], 'r-', label='thetad')
    ax.plot([x, x + l * np.cos(theta)], [y, y + l * np.sin(theta)], 'g-', label='theta')
    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)
    ax.set_aspect('equal')
    ax.grid(True)
    ax.set_title(f"Tiempo: {t:.2f} s")
    ax.legend()
    plt.pause(0.01)

    t += dt

plt.ioff()
plt.show()
