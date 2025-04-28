from roboticstoolbox import RRTPlanner, PolygonMap, Bicycle
import matplotlib.pyplot as plt
from spatialmath import Polygon2
from math import pi
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle, Polygon

# Configuración inicial
plt.close('all')

# ================== CONFIGURACIÓN DEL ENTORNO ==================

# Crear obstáculos poligonales
world_map = PolygonMap(workspace=[0, 10])
obstacles = [
    [(5, 50), (5, 6), (6, 6), (6, 50)],  # Obstáculo superior
    [(5, 4), (5, -50), (6, -50), (6, 4)]  # Obstáculo inferior
]

for obstacle in obstacles:
    world_map.add(obstacle)

# ================== CONFIGURACIÓN DEL VEHÍCULO ==================

# Parámetros del vehículo
vehicle_length = 3
vehicle_width = 1.5

# Polígono que representa el vehículo
vpolygon = Polygon2([
    (-vehicle_length/2, vehicle_width/2),
    (-vehicle_length/2, -vehicle_width/2),
    (vehicle_length/2, -vehicle_width/2),
    (vehicle_length/2, vehicle_width/2)
])

# Modelo cinemático del vehículo (bicicleta)
vehicle = Bicycle(
    steer_max=1,       # Máximo ángulo de dirección (rad)
    L=2,               # Distancia entre ejes
    polygon=vpolygon   # Forma del vehículo
)

# ================== PLANIFICACIÓN RRT ==================

# Crear planificador RRT
rrt_planner = RRTPlanner(
    map=world_map,
    vehicle=vehicle,
    npoints=200,      # Número de puntos en el espacio de configuración
    seed=42           # Semilla para reproducibilidad
)

# Configuraciones inicial y final
start_config = (2, 8, -pi/2)  # (x, y, theta)
goal_config = (8, 2, -pi/2)

# Planificar ruta
rrt_planner.plan(goal=goal_config)
path, status = rrt_planner.query(start=start_config)

print(f"Estado de la planificación: {status}")
print(f"Número de puntos en la trayectoria: {len(path)}")

# ================== VISUALIZACIÓN ==================

# Crear figura
fig, ax = plt.subplots(figsize=(10, 8))
ax.set_xlim(0, 10)
ax.set_ylim(0, 10)
ax.set_aspect('equal')
ax.grid(True)
ax.set_title('Simulación de Vehículo con Trayectoria RRT')
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')

# Dibujar obstáculos (corregido)
for obstacle in obstacles:
    ax.add_patch(Polygon(obstacle, color='gray', alpha=0.7))

# Dibujar trayectoria completa
trajectory_line, = ax.plot([], [], 'b-', lw=1, alpha=0.5, label='Trayectoria')
current_point, = ax.plot([], [], 'ro', markersize=5, label='Posición actual')

# Dibujar vehículo
vehicle_patch = ax.add_patch(Rectangle((0, 0), vehicle_length, vehicle_width, 
                                    angle=0, color='green', alpha=0.7))

# Dibujar puntos inicial y final
ax.plot(start_config[0], start_config[1], 'go', markersize=10, label='Inicio')
ax.plot(goal_config[0], goal_config[1], 'yo', markersize=10, label='Objetivo')

# Dibujar orientación inicial/final
ax.quiver(start_config[0], start_config[1], 
          np.cos(start_config[2]), np.sin(start_config[2]), 
          scale=10, color='g')
ax.quiver(goal_config[0], goal_config[1], 
          np.cos(goal_config[2]), np.sin(goal_config[2]), 
          scale=10, color='y')

ax.legend()

# ================== ANIMACIÓN ==================

def init():
    trajectory_line.set_data([], [])
    current_point.set_data([], [])
    vehicle_patch.set_xy((0, 0))
    vehicle_patch.angle = 0
    return trajectory_line, current_point, vehicle_patch

def update(frame):
    # Actualizar línea de trayectoria (hasta el frame actual)
    trajectory_line.set_data(path[:frame+1, 0], path[:frame+1, 1])
    
    # Actualizar punto actual
    current_point.set_data(path[frame, 0], path[frame, 1])
    
    # Actualizar posición y orientación del vehículo
    x, y, theta = path[frame]
    
    # Calcular esquina inferior izquierda del rectángulo
    x_corner = x - vehicle_length/2 * np.cos(theta) + vehicle_width/2 * np.sin(theta)
    y_corner = y - vehicle_length/2 * np.sin(theta) - vehicle_width/2 * np.cos(theta)
    
    vehicle_patch.set_xy((x_corner, y_corner))
    vehicle_patch.angle = np.degrees(theta)
    
    return trajectory_line, current_point, vehicle_patch

# Crear animación
ani = FuncAnimation(
    fig, update, frames=len(path),
    init_func=init, blit=True,
    interval=50, repeat=True
)

plt.tight_layout()
plt.show()