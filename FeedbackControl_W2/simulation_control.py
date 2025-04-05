import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

# Parámetros del sistema
tao = 0.001  # Paso de integración
T_sim = 10   # Tiempo de simulación

# Estados iniciales
q = np.array([[0, 0, 0]]).T  # Vector de estados q = [x, y, theta]
u = np.array([[0, 0]]).T

out = np.hstack((np.array([[0]]), q.T))

# Ganancia del control
k = 1

q_d = np.array([[5, 5]]).T  
h = 0.05

for t in np.arange(tao, T_sim, tao):
    # Error entre posición actual y deseada (solo x,y)
    e = q[:2] - q_d
    
    # Matriz de transformación para el control
    D = np.array([
        [np.cos(q[2][0]), -h*np.sin(q[2][0])],
        [np.sin(q[2][0]), h*np.cos(q[2][0])]
    ])
    
    # Control (asumiendo D es invertible, lo cual es cierto para h≠0)
    u = np.linalg.inv(D) @ (-k * e)
    
    # Matriz de evolución del sistema
    gx = np.array([
        [np.cos(q[2][0]), -h*np.sin(q[2][0])],
        [np.sin(q[2][0]), h*np.cos(q[2][0])],
        [0, 1]  
    ])
    
    q = q + tao * gx @ u

    out = np.vstack((out, np.hstack((np.array([[t]]), q.T))))

out_df = pd.DataFrame(out, columns=['t', 'x', 'y', 'theta'])

plt.figure(figsize=(6,6))
plt.plot(out_df['x'], out_df['y'], label="Trayectoria del robot")
plt.scatter(0, 0, color='red', marker='o', label="Inicio (0,0)")
plt.scatter(5, 5, color='green', marker='x', label="Meta (5,5)")
plt.title("Trayectoria del robot en el espacio")
plt.xlabel("Posición X")
plt.ylabel("Posición Y")
plt.legend()
plt.grid()
plt.axis("equal")
plt.show()