import numpy as np
import matplotlib.pyplot as plt
import heapq

# Parámetros del mapa
map_size = (20, 20)
prob_map = np.full(map_size, 0.1)  # Probabilidad inicial: libre

# Obstáculos estáticos
static_obstacles = [(5, 5), (5, 6), (5, 7), (10, 10), (10, 11)]
for x, y in static_obstacles:
    prob_map[y, x] = 0.9

# Obstáculos dinámicos (posición inicial)
dynamic_obstacles = [(8, 3), (14, 14)]

# Parámetros
start = (0, 0)
goal = (19, 19)
max_steps = 50

def is_occupied(prob):
    return prob > 0.7

def get_neighbors(pos, map_shape):
    neighbors = []
    x, y = pos
    for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
        nx, ny = x + dx, y + dy
        if 0 <= nx < map_shape[1] and 0 <= ny < map_shape[0]:
            neighbors.append((nx, ny))
    return neighbors

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star(start, goal, prob_map):
    open_set = []
    heapq.heappush(open_set, (0 + heuristic(start, goal), 0, start))
    came_from = {}
    g_score = {start: 0}
    
    while open_set:
        _, current_cost, current = heapq.heappop(open_set)
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]
        
        for neighbor in get_neighbors(current, prob_map.shape):
            if is_occupied(prob_map[neighbor[1], neighbor[0]]):
                continue
            tentative_g = current_cost + 1
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                g_score[neighbor] = tentative_g
                f = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f, tentative_g, neighbor))
                came_from[neighbor] = current
    return []

# Visualización
def plot_map(prob_map, path=None, dynamic_obs=None, step=0):
    plt.imshow(prob_map, cmap='gray', origin='lower', vmin=0, vmax=1)
    if path:
        px, py = zip(*path)
        plt.plot(px, py, 'g.-', label="Path")
    if dynamic_obs:
        dx, dy = zip(*dynamic_obs)
        plt.plot(dx, dy, 'ro', label="Dynamic")
    plt.plot(start[0], start[1], 'bo', label="Start")
    plt.plot(goal[0], goal[1], 'yo', label="Goal")
    plt.title(f"Step {step}")
    plt.legend()
    plt.pause(0.5)
    plt.clf()

# Simulación
for step in range(max_steps):
    # Limpia posiciones anteriores
    prob_map[prob_map < 0.9] = 0.1
    
    # Mueve obstáculos dinámicos
    new_dynamic = []
    for x, y in dynamic_obstacles:
        dx = np.random.choice([-1, 0, 1])
        dy = np.random.choice([-1, 0, 1])
        nx, ny = max(0, min(map_size[1]-1, x+dx)), max(0, min(map_size[0]-1, y+dy))
        new_dynamic.append((nx, ny))
        prob_map[ny, nx] = 0.95  # alta probabilidad de ocupación
    dynamic_obstacles = new_dynamic

    # Planea camino
    path = a_star(start, goal, prob_map)
    
    # Visualiza
    plot_map(prob_map, path, dynamic_obstacles, step)
    
    if path:
        start = path[1] if len(path) > 1 else path[0]
        if start == goal:
            print("Objetivo alcanzado.")
            break
    else:
        print("Sin ruta disponible en este paso.")

plt.close()
