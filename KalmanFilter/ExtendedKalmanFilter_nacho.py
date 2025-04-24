import numpy as np
import matplotlib.pyplot as plt

# Matrices y condiciones iniciales
alpha = 5
gamma = 4
beta = 0.5
delta = 3

# Obtenida de linealizar las ecuación de Lotka-Volterra
A = np.array([[alpha, 0],
              [0, -gamma]])

B = np.array([[0],
              [0]])
u = 0

# Estado inicial del sistema
x = np.array([[10], #x -> predator
              [5]]) #y -> prey

# Estado estimado inicial
x_hat = np.array([[0],
                  [0]])

# Matriz de covarianza inicial
P = np.array([[0.0, 0.0],
              [0.0, 0.0]])

# Matrices de ruido
H = np.array([[1, 0],
              [0, 1]])

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
    
    # dx/dt = alpha*x - beta*x*y
    # dy/dt = -gamma*y + delta*x*y
    f1 = x[0, 0] * (alpha - beta*x[1, 0])
    f2 = -x[1, 0] * (gamma - delta*x[0,0])
    
    x_dot = np.array([[f1],
                      [f2]]) # [f1, f2]
    Z = H @ x  # Medición


    # Integración de Euler
    x = x + dt * x_dot 
    x_hat = x_hat + dt * x_hat_dot
    P = P + dt * P_dot

    # Guardar datos
    x_real_hist.append(x[0, 0])       # Posición real
    x_hat_hist.append(x_hat[0, 0])    # Posición estimada
    t_hist.append(t)

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
