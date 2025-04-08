import numpy as np 
import matplotlib.pyplot as plt

# Configurar parámetros del sistema
dt = 1.0
F = np.array([[1, dt],
              [0, 1]])              # Modelo dinámico
H = np.array([[1, 0]])              # Observamos solo la posición
Q = np.array([[0.01, 0],
              [0, 0.1]])            # Ruido del proceso
R = np.array([[0.5]])               # Ruido de medición (solo posición)

# Inicialización
x_est = np.array([[2], [5]])        # Estimación inicial del estado
P = np.array([[0.01, 0],
              [0, 1]])              # Covarianza inicial

# Verdadero estado inicial y mediciones simuladas
x_true = x_est.copy()
num_steps = 20
true_states = []
measurements = []
estimates = []
covariances = []

np.random.seed(42)  # Para reproducibilidad

for k in range(num_steps):
    # Simular el sistema verdadero
    w = np.random.multivariate_normal([0, 0], Q).reshape(-1, 1)
    x_true = F @ x_true + w
    true_states.append(x_true.flatten())

    # Simular medición (solo posición)
    v = np.random.normal(0, np.sqrt(R[0, 0]))
    z = H @ x_true + v
    measurements.append(z.item())

    # --- Filtro de Kalman ---
    # Predicción
    x_pred = F @ x_est
    P_pred = F @ P @ F.T + Q

    # Actualización
    y = z - H @ x_pred                          # residual
    S = H @ P_pred @ H.T + R                    # innovación
    K = P_pred @ H.T @ np.linalg.inv(S)         # ganancia de Kalman
    x_est = x_pred + K @ y                      # estimación corregida
    P = (np.eye(2) - K @ H) @ P_pred            # covarianza actualizada

    estimates.append(x_est.flatten())
    covariances.append(P)

# Imprimir resultados finales
print("Estimación final del estado (media):")
print(x_est)

print("\nCovarianza final del estado:")
print(P)
