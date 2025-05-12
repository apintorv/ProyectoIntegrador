import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle

def natural_cubic_spline(x, y, x_query):
    """Calcula el spline cúbico natural (implementación manual)."""
    n = len(x) - 1
    h = np.diff(x)
    
    # Sistema de ecuaciones para M (segundas derivadas)
    A = np.zeros((n+1, n+1))
    b = np.zeros(n+1)
    
    A[0, 0] = 1  # Condición de frontera natural: M0 = 0
    A[n, n] = 1   # Condición de frontera natural: Mn = 0
    
    for i in range(1, n):
        A[i, i-1] = h[i-1]
        A[i, i] = 2 * (h[i-1] + h[i])
        A[i, i+1] = h[i]
        b[i] = 6 * ((y[i+1] - y[i]) / h[i] - (y[i] - y[i-1]) / h[i-1])
    
    M = np.linalg.solve(A, b)
    
    # Coeficientes para cada segmento
    coeffs = []
    for i in range(n):
        a0 = y[i]
        a1 = (y[i+1] - y[i]) / h[i] - (h[i] * M[i] / 2) - (h[i] * (M[i+1] - M[i]) / 6)
        a2 = M[i] / 2
        a3 = (M[i+1] - M[i]) / (6 * h[i])
        coeffs.append((a0, a1, a2, a3))
    
    # Evaluar el spline en x_query
    y_query = np.zeros_like(x_query)
    for i in range(len(x_query)):
        idx = np.searchsorted(x, x_query[i]) - 1
        idx = max(0, min(idx, n-1))
        xi = x[idx]
        a0, a1, a2, a3 = coeffs[idx]
        dx = x_query[i] - xi
        y_query[i] = a0 + a1 * dx + a2 * dx**2 + a3 * dx**3
    
    return y_query

# Datos de la trayectoria
x_nodes = np.array([0, 1, 2, 3, 4])
y_nodes = np.array([0, 1, 0, 1, 0])

# Puntos para evaluar el spline (trayectoria suave)
x_spline = np.linspace(0, 4, 100)
y_spline = natural_cubic_spline(x_nodes, y_nodes, x_spline)

# Configuración de la animación
fig, ax = plt.subplots(figsize=(10, 5))
ax.set_xlim(-0.5, 4.5)
ax.set_ylim(-0.5, 1.5)
ax.set_title('Simulación de un Carrito Moviéndose en un Spline Cúbico')
ax.set_xlabel('Posición X')
ax.set_ylabel('Posición Y')
ax.grid(True)

# Dibujar la trayectoria del spline
ax.plot(x_spline, y_spline, 'b-', label='Trayectoria (spline)')
ax.plot(x_nodes, y_nodes, 'ro', label='Nodos')

# Crear el carrito (un rectángulo + círculo)
carrito = Rectangle((0, -0.1), 0.2, 0.1, fill=True, color='red')
rueda_izq = plt.Circle((0.1, -0.1), 0.05, color='black')
rueda_der = plt.Circle((0.3, -0.1), 0.05, color='black')
ax.add_patch(carrito)
ax.add_patch(rueda_izq)
ax.add_patch(rueda_der)

# Función de inicialización
def init():
    carrito.set_xy((0, -0.1))
    rueda_izq.center = (0.1, -0.1)
    rueda_der.center = (0.3, -0.1)
    return carrito, rueda_izq, rueda_der

# Función de actualización para cada frame
def update(frame):
    x_pos = x_spline[frame]
    y_pos = y_spline[frame]
    
    # Actualizar posición del carrito y ruedas
    carrito.set_xy((x_pos - 0.1, y_pos - 0.1))
    rueda_izq.center = (x_pos - 0.05, y_pos - 0.1)
    rueda_der.center = (x_pos + 0.05, y_pos - 0.1)
    
    return carrito, rueda_izq, rueda_der

# Crear la animación
ani = FuncAnimation(
    fig, update, frames=len(x_spline), init_func=init,
    blit=True, interval=50, repeat=True
)

plt.legend()
plt.show()

# Para guardar la animación (opcional)
# ani.save('carrito_spline.mp4', writer='ffmpeg', fps=30)