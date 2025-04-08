import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import multivariate_normal

# Parámetros del modelo
dt = 1.0
F = np.array([[1, dt],
              [0, 1]])
Q = np.array([[0.01, 0],
              [0, 0.1]])

# Estados y covarianzas
mean_0 = np.array([2, 5])
cov_0 = np.array([[0.01, 0],
                  [0, 1]])

mean_1 = np.array([2.47, 19])
cov_1 = np.array([[0.04, 0.06],
                  [0.06, 0.49]])

# Mallas adaptadas a cada caso
x0, y0 = np.mgrid[1.5:3:.005, 2:8:.01]
pos0 = np.dstack((x0, y0))

x1, y1 = np.mgrid[1.5:3:.005, 17:21:.01]  # Ajuste aquí
pos1 = np.dstack((x1, y1))

# Crear distribuciones
rv_0 = multivariate_normal(mean_0, cov_0)
rv_1 = multivariate_normal(mean_1, cov_1)

pdf_0 = rv_0.pdf(pos0)
pdf_1 = rv_1.pdf(pos1)

# Graficar
fig, axes = plt.subplots(1, 2, figsize=(18, 5))

contour0 = axes[0].contourf(x0, y0, pdf_0, cmap='viridis')
axes[0].set_title('PDF of $x_0$')
axes[0].set_xlabel('Position (m)')
axes[0].set_ylabel('Velocity (m/s)')
fig.colorbar(contour0, ax=axes[0])

contour1 = axes[1].contourf(x1, y1, pdf_1, cmap='viridis')
axes[1].set_title('PDF of $x_1$')
axes[1].set_xlabel('Position (m)')
axes[1].set
