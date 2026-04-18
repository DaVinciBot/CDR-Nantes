"""
pathfinder.py
Algorithme A* optimisé pour la Raspberry Pi.
"""
import math
import heapq

class PathFinder:
    def __init__(self, terrain):
        self.terrain = terrain
        self.GRID_SIZE = 50  
        self.MARGIN = 40     
        self.inflation_radius = self.terrain.ROBOT_RADIUS + self.MARGIN

        self.cols = int(self.terrain.WIDTH / self.GRID_SIZE)
        self.rows = int(self.terrain.HEIGHT / self.GRID_SIZE)
        
        # Création de la grille statique une seule fois
        self.static_grid = [[0 for _ in range(self.rows)] for _ in range(self.cols)]
        self._build_static_grid()

    def _to_grid(self, x, y):
        return int(x / self.GRID_SIZE), int(y / self.GRID_SIZE)

    def _to_world(self, gx, gy):
        return gx * self.GRID_SIZE + self.GRID_SIZE/2, gy * self.GRID_SIZE + self.GRID_SIZE/2

    def _inflate_on_grid(self, grid, cx, cy, w, h, inflation):
        min_gx, min_gy = self._to_grid(cx - inflation, cy - inflation)
        max_gx, max_gy = self._to_grid(cx + w + inflation, cy + h + inflation)
        
        min_gx = max(0, min_gx); max_gx = min(self.cols-1, max_gx)
        min_gy = max(0, min_gy); max_gy = min(self.rows-1, max_gy)

        for x in range(min_gx, max_gx + 1):
            for y in range(min_gy, max_gy + 1):
                grid[x][y] = 1

    def _build_static_grid(self):
        """Gonfle les obstacles fixes sur la grille statique."""
        for obs in self.terrain.obstacles:
            self._inflate_on_grid(self.static_grid, obs.x, obs.y, obs.width, obs.height, self.inflation_radius)

    def create_dynamic_grid(self, lidar_obstacles):
        """Copie la grille statique et y ajoute les obstacles dynamiques."""
        grid = [row[:] for row in self.static_grid]
        
        for (lx, ly) in lidar_obstacles:
            self._inflate_on_grid(grid, lx, ly, 1, 1, self.inflation_radius)

        return grid

    def get_path(self, start_pos, end_pos, lidar_data):
        grid = self.create_dynamic_grid(lidar_data)
        
        start_node = self._to_grid(start_pos['x'], start_pos['y'])
        end_node = self._to_grid(end_pos['x'], end_pos['y'])

        if not (0 <= start_node[0] < self.cols and 0 <= start_node[1] < self.rows): return []
        if not (0 <= end_node[0] < self.cols and 0 <= end_node[1] < self.rows) or grid[end_node[0]][end_node[1]] == 1: return []

        open_set = []
        heapq.heappush(open_set, (0, start_node))
        came_from = {}
        g_score = {start_node: 0}
        closed_set = set()

        while open_set:
            current = heapq.heappop(open_set)[1]
             if current in closed_set:
                continue
            closed_set.add(current)
            if current == end_node:
                path = []
                while current in came_from:
                    wx, wy = self._to_world(current[0], current[1])
                    path.append((wx, wy))
                    current = came_from[current]
                path.reverse()

                # CORRECTIF : Ajout du point de départ en tête de chemin
                start_wx, start_wy = self._to_world(start_node[0], start_node[1])
                path.insert(0, (start_wx, start_wy))

                return path

            neighbors = [(0,1), (0,-1), (1,0), (-1,0), (1,1), (1,-1), (-1,1), (-1,-1)]
            
            for dx, dy in neighbors:
                neighbor = (current[0] + dx, current[1] + dy)
                
                if not (0 <= neighbor[0] < self.cols and 0 <= neighbor[1] < self.rows):
                    continue
                if grid[neighbor[0]][neighbor[1]] == 1:
                    continue

                # CORRECTIF : Interdit de couper un coin d'obstacle en diagonale
                if abs(dx) + abs(dy) == 2:
                    if grid[current[0] + dx][current[1]] == 1 or \
                       grid[current[0]][current[1] + dy] == 1:
                        continue

                # Coût différentiel: 1 pour tout droit, 1.414 (racine de 2) pour diagonale
                cost = 1.414 if abs(dx) + abs(dy) == 2 else 1.0
                new_score = g_score[current] + cost 
                
                if new_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = new_score
                    h = math.sqrt((neighbor[0]-end_node[0])**2 + (neighbor[1]-end_node[1])**2)
                    heapq.heappush(open_set, (new_score + h, neighbor))
                        
        return []