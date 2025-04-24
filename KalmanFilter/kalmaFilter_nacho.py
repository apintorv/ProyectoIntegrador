import numpy as np
import matplotlib.pyplot as plt

# Matrices y condiciones iniciales
A = np.array([[-1, 0],
              [0, -2]])

B = np.array([[0],
              [0.5]])
u = 0

# Estado inicial del carrito
x = np.array([[10],
              [10]])

# Estado estimado inicial
x_hat = np.array([[10],
                  [10]])

# Matriz de covarianza inicial
P = np.array([[0.0, 0.0],
              [0.0, 0.0]])

# Matrices de ruido
H = np.array([[1, 0],
              [0, 1]])

# 
R = np.array([[0.05]])

# Matriz de covarianza del proceso
Q = np.array([[0.1, 0],
              [0, 0.1]])

# Medición inicial
Z = np.array([[0],
              [0]])

# Tiempo de simulación
dt = 0.01
T_sim = 10   # Tiempo de simulación

# Para almacenar resultados
x_real_hist = []
x_hat_hist = []
t_hist = []

# Simulación
for t in np.arange(dt, T_sim, dt):
    # Ruido de proceso
    w = np.random.normal(0, 10, (2,1))

    # Kalman update
    x_hat_dot = A @ x_hat + B * u + P @ H.T * np.linalg.inv(R) @ (Z - H @ x_hat)
    P_dot = A @ P + P @ A.T + Q - P @ H.T * np.linalg.inv(R) @ H @ P
    
    # Ecuaciones del sistema
    x_dot = A @ x + B * u + w
    Z = H @ x  # Medición

    # Integración de Euler
    x = x + dt * x_dot 
    x_hat = x_hat + dt * x_hat_dot
    P = P + dt * P_dot

    # Guardar datos
    x_real_hist.append(x[0, 0])       # Posición real
    x_hat_hist.append(x_hat[0, 0])    # Posición estimada
    t_hist.append(t * dt)

# Convertir a arrays
x_real_hist = np.array(x_real_hist)
x_hat_hist = np.array(x_hat_hist)
t_hist = np.array(t_hist)

# Gráfica
plt.figure(figsize=(10, 6))
plt.plot(t_hist, x_real_hist, label='Posición real (con ruido)', linestyle='--', color='gray')
plt.plot(t_hist, x_hat_hist, label='Estimación Kalman', linewidth=2, color='blue')
plt.xlabel('Tiempo [s]')
plt.ylabel('Posición')
plt.title('Comparación de posición: Real vs Kalman')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()
