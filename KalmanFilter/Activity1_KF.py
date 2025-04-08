import numpy as np

# Paso 1: Predicción del estado
A = np.array([[1, 0.5],
              [0, 1]])
B = np.array([[0],
              [0.5]])
u = -2
x_prev = np.array([[2.47],
                   [1.19]])

x_pred = A @ x_prev + B * u  # x̂_k|k-1

# Paso 2: Predicción de la covarianza
P_prev = np.array([[0.04, 0.06],
                   [0.06, 0.49]])
Q = np.array([[0.1, 0],
              [0, 0.1]])

P_pred = A @ P_prev @ A.T + Q  # P̌_k

# Paso 3: Ganancia de Kalman
H = np.array([[1, 0]])
R = np.array([[0.05]])

S = H @ P_pred @ H.T + R
K = P_pred @ H.T @ np.linalg.inv(S)  # K_k

# Paso 4: Actualización del estado
z = np.array([[2.3]])
y = z - H @ x_pred  
x_hat = x_pred + K @ y  # x̂_k

# Paso 5: Actualización de la covarianza
I = np.eye(2)
P_hat = (I - K @ H) @ P_pred  # P̂_k

# Resultados
print("Estado estimado (x̂):")
print(x_hat)

print("\nCovarianza estimada (P̂):")
print(P_hat)
