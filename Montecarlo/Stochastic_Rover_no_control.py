##MONTECARLO SIN CONTROL
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

# Parámetros del sistema
tao = 0.01  # Paso de integración
T_sim = 3   # Tiempo de simulación

# Estados iniciales
q = np.array([[0, 0, 0]]).T  # Vector de estados q = [x, y, theta]
u = np.array([[0, 0]]).T

out = np.hstack((np.array([[0]]), q.T))

q_d = np.array([[5, 5]]).T  
h = 0.05

for n in range(10):
    q = np.array([[0,0,0]]).T
    for t in np.arange(tao, T_sim, tao):
        u = np.array([[2, 0.01]]).T
        
        w = np.array([np.random.normal(0,5, 3)]).T

        # Matriz de evolución del sistema
        gx = np.array([
            [np.cos(q[2][0]), -h*np.sin(q[2][0])],
            [np.sin(q[2][0]), h*np.cos(q[2][0])],
            [0, 1]  
        ])
        
        q = q + tao * (gx @ u + w)

        out = np.vstack((out, np.hstack((np.array([[t]]), q.T))))

out_df = pd.DataFrame(out, columns=['t', 'x', 'y', 'theta'])
out_df.plot(x= 't', y=['x','y', 'theta'], style='.')

plt.figure(figsize=(6,6))
plt.plot(out_df['x'], out_df['y'], label="Trayectoria del robot", marker = '.')
plt.scatter(0, 0, color='red', marker='o', label="Inicio (0,0)")
plt.scatter(5, 5, color='green', marker='x', label="Meta (5,5)")
plt.title("Trayectoria del robot en el espacio")
plt.xlabel("Posición X")
plt.ylabel("Posición Y")
plt.legend()
plt.grid()
plt.axis("equal")
plt.show()