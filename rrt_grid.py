import numpy as np
from roboticstoolbox import BinaryOccupancyGrid, Bug2

# Crear un grid de ocupación (0 = libre, 1 = obstáculo)
grid = BinaryOccupancyGrid(np.zeros((10, 10)), cellsize=0.5)


# Definir puntos de inicio y meta
start = (1, 1)
goal = (8, 8)

# Inicializar Bug2 (sin argumentos)
bug2 = Bug2(grid=grid, start=start, goal=goal)
# Planificar la trayectoria
path = bug2.plan()
# Imprimir la trayectoria
print("Path:", path)

