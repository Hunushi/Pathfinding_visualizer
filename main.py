import pygame
import random
import collections
import heapq


class Gui():
    '''
    Classe pour l'interface graphique de l'application
    '''
    
    # Constantes GUI
    FPS = 60
    WIDTH = 800
    MIN_GRID_SIZE = 5
    MAX_GRID_SIZE = 100
    
    # Couleurs
    COLOR_WHITE = (255, 255, 255)
    COLOR_BLACK = (0, 0, 0)
    COLOR_RED = (255, 0, 0)
    COLOR_GREEN = (0, 255, 0)
    COLOR_BLUE = (0, 0, 255)
    COLOR_LIGHT_BLUE = (135, 206, 250)
    COLOR_ORANGE = (255, 165, 0)
    COLOR_PURPLE = (255, 0, 255)

    def __init__(self, coords):
        # Variables GUI
        self.grid_size = 20
        self.box_width = self.WIDTH / self.grid_size
        self.coords = coords
        self.placing_walls = False
        self.removing_walls = False
        self.animation_speed = 1
        self.algorithm_running = False

        self.coords.maze = [
            [0 for x in range(self.grid_size)] for y in range(self.grid_size)]

        # Démarrage de l'application pygame
        pygame.init()
        self.win = pygame.display.set_mode((self.WIDTH, self.WIDTH))
        self.clock = pygame.time.Clock()
        pygame.display.set_caption("Algorithmes de Pathfinding")

    def main(self, running=False):
        self.clock.tick(self.FPS)
        self.mouse_x, self.mouse_y = pygame.mouse.get_pos()
                
        # Si le bouton de la souris est pressé, continuer à placer/enlever des murs
        if not running and not self.algorithm_running:
            if self.placing_walls == True:
                self.place_wall()
            elif self.removing_walls == True:
                self.remove()

        # Gérer les événements souris et clavier
        self.event_handle(running)

        # Redessiner et mettre à jour l'affichage
        self.redraw()
        pygame.display.update()

    def event_handle(self, running):
        run_keys = {"q", "w", "e", "r"}

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()

            elif event.type == pygame.KEYDOWN:
                if self.algorithm_running:
                    # Pendant l'exécution d'un algorithme, seules certaines touches fonctionnent
                    if event.key == pygame.K_r or event.key == pygame.K_z:
                        print("Réinitialisation de la visualisation...")
                        self.coords.remove_last()
                        self.algorithm_running = False
                    elif event.key == pygame.K_ESCAPE or event.key == pygame.K_q:
                        pygame.quit()
                        exit()
                    continue

                key = chr(event.key)

                if running == False and not self.algorithm_running:
                    # Lancer l'algorithme
                    if key in run_keys:  # q, w, e et r
                        self.run_algorithm(key)
                    
                    # Effacer tout le plateau
                    elif key == "x" or key == "c":
                        self.coords.remove_all()
                        self.algorithm_running = False
                        
                    # Enlever tout sauf ce qui a été placé par l'utilisateur
                    elif key == "z":
                        self.coords.remove_last()
                        self.algorithm_running = False

                    # Générer un labyrinthe aléatoire
                    elif key == " ":
                        self.coords.generate_random_maze(self)

                # Augmenter la vitesse du pathfinding
                if (key == "+" or key == "=") and self.animation_speed > 1:
                    self.animation_speed = max(1, int(self.animation_speed * 0.5))

                # Diminuer la vitesse du pathfinding
                elif key == "-":
                    self.animation_speed = int(self.animation_speed * 2) + 1

            elif event.type == pygame.MOUSEBUTTONDOWN:
                if self.algorithm_running:
                    continue

                coords = self.get_box_coords()
                if coords:
                    if event.button == 1:  # Clic gauche
                        if self.coords.start is None:
                            self.set_start_point(coords)
                        elif self.coords.end is None:
                            self.set_end_point(coords)
                        else:
                            self.placing_walls = True
                            self.place_wall()

                    elif event.button == 3:  # Clic droit
                        self.removing_walls = True
                        self.remove()

                    elif event.button == 4:  # Molette haut - zoom in
                        if self.grid_size < self.MAX_GRID_SIZE:
                            self.grid_size += 1
                            self.update_dimensions()

                    elif event.button == 5:  # Molette bas - zoom out
                        if self.grid_size > self.MIN_GRID_SIZE:
                            self.grid_size -= 1
                            self.update_dimensions()

            elif event.type == pygame.MOUSEMOTION:
                if self.placing_walls and pygame.mouse.get_pressed()[0]:
                    self.place_wall()
                elif self.removing_walls and pygame.mouse.get_pressed()[2]:
                    self.remove()

            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:
                    self.placing_walls = False
                elif event.button == 3:
                    self.removing_walls = False

    def update_dimensions(self):
        """Recalcule les dimensions après un zoom"""
        self.box_width = self.WIDTH / self.grid_size
        self.coords.create_maze(self)
        self.coords.remove_last()

    def redraw(self):
        """Redessine l'interface"""
        self.win.fill(self.COLOR_WHITE)
        self.draw_points()
        self.draw_grid()

    def draw_grid(self):
        """Dessine les lignes de la grille"""
        for i in range(self.grid_size - 1):
            pygame.draw.rect(self.win, self.COLOR_BLACK,
                             (((i+1)*self.box_width)-2, 0, 4, self.WIDTH))
            pygame.draw.rect(self.win, self.COLOR_BLACK,
                             (0, ((i+1)*self.box_width)-2, self.WIDTH, 4))

    def draw_points(self):
        """Dessine tous les carrés pour les murs, points, etc."""
        # Dessiner les nœuds de la liste fermée (explorés) - Orange
        for node in self.coords.closed_list:
            if node.position != self.coords.start and node.position != self.coords.end:
                self.draw_box(node.position, self.COLOR_ORANGE)

        # Dessiner les nœuds de la liste ouverte (à explorer) - Bleu clair
        for node in self.coords.open_list:
            if node.position != self.coords.start and node.position != self.coords.end:
                self.draw_box(node.position, self.COLOR_LIGHT_BLUE)

        # Dessiner le chemin final - Violet
        for point in self.coords.final_path:
            if point != self.coords.start and point != self.coords.end:
                self.draw_box(point, self.COLOR_PURPLE)

        # Dessiner les murs - Noir
        for wall in self.coords.walls:
            self.draw_box(wall, self.COLOR_BLACK)
        
        # Dessiner le point de départ - Rouge avec "S"
        if self.coords.start != None:
            self.draw_box(self.coords.start, self.COLOR_RED)
            self.display_text("S", self.COLOR_WHITE,
                              self.box_center(self.coords.start), int(self.box_width))
            
        # Dessiner le point d'arrivée - Vert avec "E"
        if self.coords.end != None:
            self.draw_box(self.coords.end, self.COLOR_GREEN)
            self.display_text("E", self.COLOR_WHITE,
                              self.box_center(self.coords.end), int(self.box_width))

    def box_center(self, box):
        """Obtient le point central d'un nœud"""
        boxX, boxY = box
        center = ((boxX*self.box_width+(self.box_width/2)),
                  (boxY*self.box_width+(self.box_width/2)))
        return center

    def draw_box(self, box, colour):
        """Dessine une boîte avec une couleur et une position données"""
        boxX, boxY = box
        if 0 <= boxX < self.grid_size and 0 <= boxY < self.grid_size:
            pygame.draw.rect(self.win, colour,
                            (boxX*self.box_width, boxY*self.box_width,
                             self.box_width, self.box_width))

    def get_box_coords(self):
        """Obtient les coordonnées de la boîte à partir de la position de la souris"""
        if not (0 <= self.mouse_x < self.WIDTH and 0 <= self.mouse_y < self.WIDTH):
            return None
        boxX = int((self.mouse_x + 2) / self.box_width)
        boxY = int((self.mouse_y + 2) / self.box_width)
        boxX = max(0, min(boxX, self.grid_size - 1))
        boxY = max(0, min(boxY, self.grid_size - 1))
        return (boxX, boxY)

    def set_start_point(self, coords):
        """Définit le point de départ"""
        if (coords != self.coords.end and coords not in self.coords.walls and
            0 <= coords[0] < self.grid_size and 0 <= coords[1] < self.grid_size):
            self.coords.start = coords
            self.coords.remove_last()

    def set_end_point(self, coords):
        """Définit le point d'arrivée"""
        if (coords != self.coords.start and coords not in self.coords.walls and
            0 <= coords[0] < self.grid_size and 0 <= coords[1] < self.grid_size):
            self.coords.end = coords
            self.coords.remove_last()

    def place_wall(self):
        """Place un mur"""
        coords = self.get_box_coords()
        if coords and (coords != self.coords.start and coords != self.coords.end
                and coords not in self.coords.walls):
            self.coords.walls.append(coords)
            self.coords.create_maze(self)
            self.coords.remove_last()

    def remove(self):
        """Supprime des nœuds comme les murs, points, etc."""
        coords = self.get_box_coords()
        if coords:
            if coords in self.coords.walls:
                self.coords.walls.remove(coords)
                self.coords.create_maze(self)
                self.coords.remove_last()
            elif coords == self.coords.start:
                self.coords.start = None
                self.coords.remove_last()
            elif coords == self.coords.end:
                self.coords.end = None
                self.coords.remove_last()

    def run_algorithm(self, key):
        """Prépare et lance l'algorithme de pathfinding"""
        if self.algorithm_running:
            print("Un algorithme est déjà en cours d'exécution.")
            return
        
        if not self.coords.start or not self.coords.end:
            print("Veuillez définir les points de départ et d'arrivée.")
            return

        self.placing_walls = False
        self.removing_walls = False
        self.coords.remove_last()
        self.coords.create_maze(self)
        self.algorithm_running = True

        # Lancer l'algorithme approprié
        if key == "q":  # DFS
            pathfind_dfs(self.coords.maze, self.coords.start, self.coords.end, self, self.coords)
        elif key == "w":  # BFS
            pathfind_bfs(self.coords.maze, self.coords.start, self.coords.end, self, self.coords)
        elif key == "e":  # Dijkstra
            pathfind_dijkstra(self.coords.maze, self.coords.start, self.coords.end, self, self.coords)
        elif key == "r":  # A*
            pathfind_a_star(self.coords.maze, self.coords.start, self.coords.end, self, self.coords)

        self.algorithm_running = False

    def display_text(self, txt, colour, center, size):
        """Affiche du texte avec une couleur, position et taille données"""
        font = pygame.font.Font(None, size)
        text_surf = font.render(txt, True, colour)
        text_rect = text_surf.get_rect()
        text_rect.center = center
        self.win.blit(text_surf, text_rect)


class CoOrdinates():
    '''
    Classe contenant toutes les coordonnées et fonctions pour les calculs associés
    '''

    def __init__(self):
        self.remove_all()

    def remove_all(self):
        """Réinitialise tous les éléments"""
        self.start = None
        self.end = None
        self.walls = []
        self.maze = []
        self.open_list = []
        self.closed_list = []
        self.final_path = []

    def remove_last(self):
        """Supprime seulement les éléments de visualisation"""
        self.open_list = []
        self.closed_list = []
        self.final_path = []

    def largest_distance(self):
        """Obtient la distance la plus éloignée d'un nœud depuis (0, 0)"""
        largest = 0
        for wall in self.walls:
            if wall[0] > largest: largest = wall[0]
            if wall[1] > largest: largest = wall[1]
        if self.start:
            if self.start[0] > largest: largest = self.start[0]
            if self.start[1] > largest: largest = self.start[1]
        if self.end:
            if self.end[0] > largest: largest = self.end[0]
            if self.end[1] > largest: largest = self.end[1]
        return largest + 1

    def create_maze(self, gui):
        """Crée un tableau 2D du labyrinthe et de ses murs"""
        largest_distance = self.largest_distance()
        
        if gui.grid_size > largest_distance:
            largest = gui.grid_size
        else:
            largest = largest_distance
            
        self.maze = [[0 for x in range(largest)] for y in range(largest)]
        for wall in self.walls:
            try:
                wall_x, wall_y = wall
                if 0 <= wall_x < largest and 0 <= wall_y < largest:
                    self.maze[wall_x][wall_y] = 1
            except:
                pass

    def generate_random_maze(self, gui):
        """Crée un labyrinthe aléatoire"""
        self.walls = []
        for i in range(gui.grid_size * gui.grid_size):
            if random.random() > 0.6:
                wall = (random.randint(0, gui.grid_size-1),
                        random.randint(0, gui.grid_size-1))
                if wall not in self.walls and wall != self.start and wall != self.end:
                    self.walls.append(wall)
        self.create_maze(gui)


class Node():
    '''
    Classe de nœud pour contenir la position, le parent et les coûts
    '''
    
    def __init__(self, parent, position):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

    def __lt__(self, other):
        if self.f != other.f:
            return self.f < other.f
        return self.g < other.g


def pathfind_bfs(maze, start, end, gui, coords):
    """Algorithme BFS (Breadth-First Search)"""
    start_node = Node(None, start)
    end_node = Node(None, end)

    open_list = collections.deque([start_node])
    closed_set = set()

    coords.open_list = list(open_list)
    coords.closed_list = []
    coords.final_path = []
    gui.redraw()
    pygame.display.update()

    steps = 0
    path_found = False

    while open_list:
        if steps >= gui.animation_speed:
            steps = 0
            coords.open_list = list(open_list)
            coords.closed_list = [Node(None, pos) for pos in closed_set]
            gui.redraw()
            pygame.display.update()

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return None
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE or event.key == pygame.K_r or event.key == pygame.K_z:
                        return None

        current_node = open_list.popleft()

        if current_node.position in closed_set:
            continue
        closed_set.add(current_node.position)

        if current_node.position == end:
            path_found = True
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            path.reverse()

            coords.final_path = path
            coords.open_list = []
            coords.closed_list = [Node(None, pos) for pos in closed_set]
            gui.redraw()
            pygame.display.update()
            print("Chemin trouvé via BFS!")
            return path

        for move in [(-1, 0), (0, 1), (1, 0), (0, -1)]:
            neighbor_pos = (current_node.position[0] + move[0],
                            current_node.position[1] + move[1])

            if not (0 <= neighbor_pos[0] < len(maze) and 0 <= neighbor_pos[1] < len(maze[0])):
                continue
            if maze[neighbor_pos[0]][neighbor_pos[1]] == 1:
                continue
            if neighbor_pos in closed_set:
                continue

            neighbor_node = Node(current_node, neighbor_pos)
            in_open = False
            for open_node in open_list:
                if neighbor_node == open_node:
                    in_open = True
                    break

            if not in_open:
                neighbor_node.g = current_node.g + 1
                neighbor_node.f = neighbor_node.g
                open_list.append(neighbor_node)

        steps += 1

    if not path_found:
        print("Aucun chemin trouvé via BFS.")
        coords.open_list = []
        coords.closed_list = [Node(None, pos) for pos in closed_set]
        gui.redraw()
        pygame.display.update()
    return None


def pathfind_dfs(maze, start, end, gui, coords):
    """Algorithme DFS (Depth-First Search)"""
    start_node = Node(None, start)
    end_node = Node(None, end)

    open_list = [start_node]
    closed_set = set()

    coords.open_list = list(open_list)
    coords.closed_list = []
    coords.final_path = []
    gui.redraw()
    pygame.display.update()

    steps = 0
    path_found = False

    while open_list:
        if steps >= gui.animation_speed:
            steps = 0
            coords.open_list = list(open_list)
            coords.closed_list = [Node(None, pos) for pos in closed_set]
            gui.redraw()
            pygame.display.update()

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return None
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE or event.key == pygame.K_r or event.key == pygame.K_z:
                        return None

        current_node = open_list.pop()

        if current_node.position in closed_set:
            continue
        closed_set.add(current_node.position)

        if current_node.position == end:
            path_found = True
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            path.reverse()

            coords.final_path = path
            coords.open_list = []
            coords.closed_list = [Node(None, pos) for pos in closed_set]
            gui.redraw()
            pygame.display.update()
            print("Chemin trouvé via DFS!")
            return path

        for move in [(-1, 0), (0, 1), (1, 0), (0, -1)]:
            neighbor_pos = (current_node.position[0] + move[0],
                            current_node.position[1] + move[1])

            if not (0 <= neighbor_pos[0] < len(maze) and 0 <= neighbor_pos[1] < len(maze[0])):
                continue
            if maze[neighbor_pos[0]][neighbor_pos[1]] == 1:
                continue
            if neighbor_pos in closed_set:
                continue

            already_in_open = False
            for node_in_stack in open_list:
                if neighbor_pos == node_in_stack.position:
                    already_in_open = True
                    break

            if not already_in_open:
                neighbor_node = Node(current_node, neighbor_pos)
                open_list.append(neighbor_node)

        steps += 1

    if not path_found:
        print("Aucun chemin trouvé via DFS.")
        coords.open_list = []
        coords.closed_list = [Node(None, pos) for pos in closed_set]
        gui.redraw()
        pygame.display.update()
    return None


def pathfind_dijkstra(maze, start, end, gui, coords):
    """Algorithme de Dijkstra"""
    start_node = Node(None, start)
    start_node.g = 0
    start_node.f = start_node.g

    end_node = Node(None, end)

    open_list = [(start_node.g, start_node)]
    heapq.heapify(open_list)

    g_costs = {start_node.position: 0}
    predecessors = {start_node.position: None}
    closed_set = set()

    coords.open_list = [node for cost, node in open_list]
    coords.closed_list = []
    coords.final_path = []
    gui.redraw()
    pygame.display.update()

    steps = 0
    path_found = False

    while open_list:
        if steps >= gui.animation_speed:
            steps = 0
            coords.open_list = [node for cost, node in open_list]
            coords.closed_list = [Node(None, pos) for pos in closed_set]
            gui.redraw()
            pygame.display.update()

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return None
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE or event.key == pygame.K_r or event.key == pygame.K_z:
                        return None

        current_g, current_node = heapq.heappop(open_list)

        if current_g > g_costs.get(current_node.position, float('inf')):
            continue

        if current_node.position == end:
            path_found = True
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = predecessors.get(current.position)
            path.reverse()

            coords.final_path = path
            coords.open_list = []
            coords.closed_list = [Node(None, pos) for pos in closed_set]
            gui.redraw()
            pygame.display.update()
            print("Chemin trouvé via Dijkstra!")
            return path

        closed_set.add(current_node.position)

        for move in [(-1, 0), (0, 1), (1, 0), (0, -1)]:
            neighbor_pos = (current_node.position[0] + move[0],
                            current_node.position[1] + move[1])

            if not (0 <= neighbor_pos[0] < len(maze) and 0 <= neighbor_pos[1] < len(maze[0])):
                continue
            if maze[neighbor_pos[0]][neighbor_pos[1]] == 1:
                continue
            if neighbor_pos in closed_set:
                continue

            tentative_g = current_node.g + 1

            if tentative_g < g_costs.get(neighbor_pos, float('inf')):
                new_neighbor = Node(current_node, neighbor_pos)
                new_neighbor.g = tentative_g
                new_neighbor.f = tentative_g

                g_costs[neighbor_pos] = tentative_g
                predecessors[neighbor_pos] = current_node
                heapq.heappush(open_list, (new_neighbor.f, new_neighbor))

        steps += 1

    if not path_found:
        print("Aucun chemin trouvé via Dijkstra.")
        coords.open_list = []
        coords.closed_list = [Node(None, pos) for pos in closed_set]
        gui.redraw()
        pygame.display.update()
    return None


def pathfind_a_star(maze, start, end, gui, coords):
    """Algorithme A*"""
    start_node = Node(None, start)
    start_node.g = 0
    start_node.h = abs(start_node.position[0] - end[0]) + abs(start_node.position[1] - end[1])
    start_node.f = start_node.g + start_node.h

    end_node = Node(None, end)

    open_list = [(start_node.f, start_node)]
    heapq.heapify(open_list)

    g_costs = {start_node.position: 0}
    predecessors = {start_node.position: None}
    closed_set = set()

    coords.open_list = [node for cost, node in open_list]
    coords.closed_list = []
    coords.final_path = []
    gui.redraw()
    pygame.display.update()

    steps = 0
    path_found = False

    while open_list:
        if steps >= gui.animation_speed:
            steps = 0
            coords.open_list = [node for cost, node in open_list]
            coords.closed_list = [Node(None, pos) for pos in closed_set]
            gui.redraw()
            pygame.display.update()

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return None
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE or event.key == pygame.K_r or event.key == pygame.K_z:
                        return None

        current_f, current_node = heapq.heappop(open_list)

        if current_node.position in g_costs and current_node.g > g_costs[current_node.position]:
            continue

        if current_node.position == end:
            path_found = True
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = predecessors.get(current.position)
            path.reverse()

            coords.final_path = path
            coords.open_list = []
            coords.closed_list = [Node(None, pos) for pos in closed_set]
            gui.redraw()
            pygame.display.update()
            print("Chemin trouvé via A*!")
            return path

        closed_set.add(current_node.position)

        for move in [(-1, 0), (0, 1), (1, 0), (0, -1)]:
            neighbor_pos = (current_node.position[0] + move[0],
                            current_node.position[1] + move[1])

            if not (0 <= neighbor_pos[0] < len(maze) and 0 <= neighbor_pos[1] < len(maze[0])):
                continue
            if maze[neighbor_pos[0]][neighbor_pos[1]] == 1:
                continue
            if neighbor_pos in closed_set:
                continue

            tentative_g = current_node.g + 1

            if tentative_g < g_costs.get(neighbor_pos, float('inf')):
                new_neighbor = Node(current_node, neighbor_pos)
                new_neighbor.g = tentative_g
                new_neighbor.h = abs(neighbor_pos[0] - end[0]) + abs(neighbor_pos[1] - end[1])
                new_neighbor.f = new_neighbor.g + new_neighbor.h

                g_costs[neighbor_pos] = tentative_g
                predecessors[neighbor_pos] = current_node
                heapq.heappush(open_list, (new_neighbor.f, new_neighbor))

        steps += 1

    if not path_found:
        print("Aucun chemin trouvé via A*.")
        coords.open_list = []
        coords.closed_list = [Node(None, pos) for pos in closed_set]
        gui.redraw()
        pygame.display.update()
    return None


# Boucle principale
if __name__ == "__main__":
    print("--- Contrôles ---")
    print("Clic gauche (1er): Définir le point de départ")
    print("Clic gauche (2ème): Définir le point d'arrivée")
    print("Clic gauche + Glisser: Dessiner des murs")
    print("Clic droit + Glisser: Effacer des murs")
    print("Molette: Zoom In/Out")
    print("")
    print("Touche 'q': Lancer DFS")
    print("Touche 'w': Lancer BFS")
    print("Touche 'e': Lancer Dijkstra")
    print("Touche 'r': Lancer A*")
    print("Touche 'x' ou 'c': Tout effacer")
    print("Touche 'z' ou 'r': Réinitialiser la visualisation")
    print("Touche 'espace': Générer un labyrinthe aléatoire")
    print("Touche '+': Augmenter la vitesse")
    print("Touche '-': Diminuer la vitesse")
    print("Touche 'ESC': Quitter")
    print("----------------")
    
    gui = Gui(CoOrdinates())
    while True:
        gui.main()