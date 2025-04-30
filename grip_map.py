import numpy as np
import matplotlib.pyplot as plt
from bresenham import bresenham 
from math import log, exp

class TemporalFilterOccupancyGrid:
    def __init__(self, width=10, height=10, prior=0.5, temporal_window=5):
        self.width = width
        self.height = height
        self.prior = prior
        self.grid = np.ones((height, width)) * prior
        self.observation_count = np.zeros((height, width))
        self.temporal_window = temporal_window
        self.observation_history = []
        
    def update_map(self, sensor_origin, sensor_endpoints):
        """
        Actualiza el mapa con filtrado temporal
        sensor_origin: (x,y) posición del robot
        sensor_endpoints: lista de (x,y) puntos detectados como ocupados
        """
        # Almacenar observación actual
        current_obs = np.zeros((self.height, self.width))
        
        for endpoint in sensor_endpoints:
            # Obtener celdas en la línea usando Bresenham
            cells = list(bresenham(sensor_origin[0], sensor_origin[1], endpoint[0], endpoint[1]))
            
            # Todas las celdas en la línea son libres (excepto el endpoint)
            for x, y in cells[:-1]:
                if 0 <= x < self.width and 0 <= y < self.height:
                    current_obs[y, x] = 0.1  # Probabilidad baja (libre)
            
            # El endpoint es ocupado
            x, y = cells[-1]
            if 0 <= x < self.width and 0 <= y < self.height:
                current_obs[y, x] = 0.9  # Probabilidad alta (ocupado)
        
        # Agregar a historial y mantener sólo las últimas N observaciones
        self.observation_history.append(current_obs)
        if len(self.observation_history) > self.temporal_window:
            self.observation_history.pop(0)
        
        # Actualizar mapa basado en observaciones consistentes
        self.grid = np.ones((self.height, self.width)) * self.prior
        for obs in self.observation_history:
            self.grid = np.where(obs > 0, (self.grid + obs) / 2, self.grid)
        
    def display(self):
        plt.imshow(self.grid, cmap='binary_r', vmin=0, vmax=1, origin='lower')
        plt.colorbar(label='Probability of occupancy')
        plt.title('Temporal Filter Occupancy Grid')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.show()


class BayesianOccupancyGrid:
    def __init__(self, width=10, height=10, prior=0.5):
        self.width = width
        self.height = height
        self.prior = prior
        # Usamos log-odds para estabilidad numérica
        self.grid = np.ones((height, width)) * self.logodds(prior)
        
    def logodds(self, p):
        """Calcula log-odds con protección contra división por cero"""
        p = np.clip(p, 0.001, 0.999)  # Evita valores extremos
        return log(p / (1 - p))
    
    def probability(self, l):
        """Convierte log-odds de vuelta a probabilidad"""
        return 1 - (1 / (1 + exp(l)))
    
    def update_map(self, sensor_origin, sensor_endpoints):
        """
        Actualización bayesiana usando log-odds
        """
        for endpoint in sensor_endpoints:
            cells = list(bresenham(sensor_origin[0], sensor_origin[1], endpoint[0], endpoint[1]))
            
            # Parámetros del modelo del sensor (pueden ajustarse)
            p_free = 0.3  # Probabilidad si celda está libre
            p_occ = 0.7   # Probabilidad si celda está ocupada
            
            # Usamos el prior de la clase en lugar de prior_odds
            l_free = self.logodds(p_free) - self.logodds(self.prior)
            l_occ = self.logodds(p_occ) - self.logodds(self.prior)
            
            for x, y in cells[:-1]:
                if 0 <= x < self.width and 0 <= y < self.height:
                    self.grid[y, x] += l_free
            
            x, y = cells[-1]
            if 0 <= x < self.width and 0 <= y < self.height:
                self.grid[y, x] += l_occ
    
    def get_probability_grid(self):
        return np.vectorize(self.probability)(self.grid)
    
    def display(self):
        prob_grid = self.get_probability_grid()
        plt.imshow(prob_grid, cmap='binary_r', vmin=0, vmax=1, origin='lower')
        plt.colorbar(label='Probability of occupancy')
        plt.title('Bayesian Occupancy Grid')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.show()

# Datos de ejemplo
robot_pose = (5, 5)  # Posición del robot
detections = [(7, 7), (3, 8), (6, 3)]  # Puntos detectados como ocupados

# 1. Uso del filtro temporal
print("Filtrado Temporal:")
temporal_map = TemporalFilterOccupancyGrid(width=10, height=10)
for _ in range(5):  # Simular múltiples actualizaciones
    temporal_map.update_map(robot_pose, detections)
temporal_map.display()

# 2. Uso del enfoque bayesiano
print("\nActualización Bayesiana:")
bayesian_map = BayesianOccupancyGrid(width=10, height=10)
for _ in range(5):  # Múltiples actualizaciones
    bayesian_map.update_map(robot_pose, detections)
bayesian_map.display()