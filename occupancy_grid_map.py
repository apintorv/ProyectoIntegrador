import numpy as np
import matplotlib.pyplot as plt

def create_occupancy_map(lidar_coords, map_size=(5, 5), max_range=1.0, decay_rate=0.5):
    """
    Crea un mapa de probabilidad de ocupación a partir de coordenadas LIDAR.
    
    Args:
        lidar_coords: Lista de coordenadas (x, y) detectadas por el LIDAR
        map_size: Tamaño del mapa (filas, columnas)
        max_range: Valor máximo de probabilidad para celdas ocupadas
        decay_rate: Tasa de decaimiento para celdas vecinas
        
    Returns:
        Matriz numpy representando el mapa de ocupación
    """
    occupancy_map = np.zeros(map_size)
    
    for (x, y) in lidar_coords:
        # Convertir coordenadas a índices de matriz (asumiendo coordenadas enteras)
        i, j = int(y), int(x)
        
        # Verificar que los índices estén dentro del mapa
        if 0 <= i < map_size[0] and 0 <= j < map_size[1]:
            # Asignar máxima probabilidad a la celda detectada
            occupancy_map[i, j] = max(occupancy_map[i, j], max_range)
            
            # Propagación a celdas vecinas con decaimiento
            for di in [-1, 0, 1]:
                for dj in [-1, 0, 1]:
                    if di == 0 and dj == 0:
                        continue  # Saltar la celda central ya asignada
                    ni, nj = i + di, j + dj
                    if 0 <= ni < map_size[0] and 0 <= nj < map_size[1]:
                        new_val = max_range * (1.0 - decay_rate)
                        occupancy_map[ni, nj] = max(occupancy_map[ni, nj], new_val)
    
    return occupancy_map

# Ejemplo de uso con coordenadas que producen un mapa similar a tu imagen
lidar_detections = [(1, 2), (2, 2), (1, 1), (2, 1)]  # (x, y) coordinates

# Crear el mapa
occupancy_map = create_occupancy_map(lidar_detections)

# Visualizar el mapa
plt.figure(figsize=(6, 6))
plt.imshow(occupancy_map, cmap='gray_r', vmin=0, vmax=1, origin='lower')
plt.colorbar(label='Probability of occupancy')
plt.title('Occupancy Probability Map')

# Numerar los ejes correctamente
plt.xticks(np.arange(0, 5, 1), np.arange(0, 5, 1))
plt.yticks(np.arange(0, 5, 1), np.arange(0, 5, 1))
plt.xlabel('X coordinate')
plt.ylabel('Y coordinate')

# Mostrar valores en cada celda
for i in range(occupancy_map.shape[0]):
    for j in range(occupancy_map.shape[1]):
        plt.text(j, i, f"{occupancy_map[i, j]:.1f}", ha='center', va='center', color='red')

plt.grid(which='both', color='lightgray', linestyle='-', linewidth=0.5)
plt.show()

# Imprimir la matriz
print("Matriz de probabilidad de ocupación:")
print(occupancy_map)