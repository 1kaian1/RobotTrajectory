#!/usr/bin/env python3

import rospy
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid
import numpy as np
import matplotlib.pyplot as plt
import heapq
import math

def getMap() -> OccupancyGrid:
    """ Loads map from map service """
    rospy.wait_for_service('static_map')
    try:
        get_map = rospy.ServiceProxy('static_map', GetMap)
        recMap = get_map().map
        return recMap
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        return None

def grid_to_world(row, col, origin_x, origin_y, resolution):
    """Převede indexy buňky na světové souřadnice"""
    x = origin_x + (col + 0.5) * resolution
    y = origin_y + (row + 0.5) * resolution
    return x, y

MOVES = [(1, 0),  (-1, 0),  (0, 1),  (0, -1)]

def dijkstra_grid(grid, start_cell, goal_cell, costmap, alpha=10.0):
    
    total_rows, total_cols = grid.shape

    # minimální vzdálenost do každé buňky
    distance = {start_cell: 0}

    # rodič každé buňky na nejkratší cestě
    previous_cell = {start_cell: None}

    # prioritní fronta: (vzdálenost, buňka)
    priority_queue = [(0, start_cell)]

    while priority_queue:
        current_distance, current_cell = heapq.heappop(priority_queue)
        current_row, current_col = current_cell

        # konec — došli jsme do cíle
        if current_cell == goal_cell:
            break

        # pokud je v queue starý záznam, přeskočíme ho
        if current_distance > distance[current_cell]:
            continue

        # projdeme všechny možné směry
        for move_row, move_col in MOVES:
            neighbor_row = current_row + move_row
            neighbor_col = current_col + move_col
            neighbor_cell = (neighbor_row, neighbor_col)

            # mimo mapu
            if not (0 <= neighbor_row < total_rows and 0 <= neighbor_col < total_cols):
                continue

            # překážka
            if grid[neighbor_row, neighbor_col] == 1:
                continue

            # realistická cena pohybu:
            # diagonála = sqrt(2), rovně = 1
            if move_row != 0 and move_col != 0: ## pryč
                move_cost = math.sqrt(2)        ## pryč
            else:
                move_cost = 1

            cost = move_cost + alpha * costmap[neighbor_row, neighbor_col]  # zde se přidává costmap

            new_distance = current_distance + cost

            # pokud je to kratší cesta k sousedovi, uložíme ji
            if neighbor_cell not in distance or new_distance < distance[neighbor_cell]:
                distance[neighbor_cell] = new_distance
                previous_cell[neighbor_cell] = current_cell
                heapq.heappush(priority_queue, (new_distance, neighbor_cell))

    return distance, previous_cell


def reconstruct_path(previous_cell, goal_cell):
    path = []
    cell = goal_cell
    while cell is not None:
        path.append(cell)
        cell = previous_cell.get(cell)
    return path[::-1]

def transform(x, y):
    return (6*y + 4, 6*x + 4)
    
def compute_costmap_8neighbors(binary_grid):
    """
    binary_grid: 2D numpy array, hodnoty 0 (free) nebo 1 (occupied)
    return: costmap s hodnotami 0.0 – 1.0
    """
    h, w = binary_grid.shape
    costmap = np.zeros((h, w), dtype=np.float32)

    for r in range(h):
        for c in range(w):
            neighbor_sum = 0
            neighbor_count = 0

            for dr in [-1, 0, 1]:
                for dc in [-1, 0, 1]:
                    if dr == 0 and dc == 0:
                        continue  # přeskoč střed

                    nr = r + dr
                    nc = c + dc

                    if 0 <= nr < h and 0 <= nc < w:
                        neighbor_sum += binary_grid[nr, nc]
                        neighbor_count += 1

            if neighbor_count > 0:
                costmap[r, c] = neighbor_sum / neighbor_count
            else:
                costmap[r, c] = 0.0

    return costmap


def bresenham_line(x1, y1, x2, y2):
    """Vrací seznam buněk mezi dvěma body podle Bresenhama."""
    points = []
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    x, y = x1, y1
    sx = 1 if x2 > x1 else -1
    sy = 1 if y2 > y1 else -1

    if dx >= dy:
        err = dx // 2
        while x != x2:
            points.append((x, y))
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy // 2
        while y != y2:
            points.append((x, y))
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy

    points.append((x2, y2))
    return points


def can_see(a, b, costmap, threshold=0.133):
    """
    Vrací True, pokud všechny body mezi a a b mají hodnotu ≤ threshold.
    a, b = (row, col)
    """
    for r, c in bresenham_line(a[0], a[1], b[0], b[1]):
        if costmap[r, c] > threshold:
            return False
    return True


def line_of_sight_smooth(path, costmap, threshold=0.133):
    """
    Očistí cestu od zbytečných bodů.
    Vrací novou, hladší cestu.
    """
    if len(path) <= 2:
        return path.copy()

    smoothed = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        # najdi nejvzdálenější viditelný bod
        while j > i and not can_see(path[i], path[j], costmap, threshold):
            j -= 1
        smoothed.append(path[j])
        i = j
    return smoothed



if __name__ == "__main__":
    # Inicializace ROS node
    rospy.init_node('moro_maze_navigation', anonymous=True)

    # Získání mapy
    recMap = getMap()
    if recMap is None:
        exit(1)

    # Parametry mapy
    width = recMap.info.width
    height = recMap.info.height
    resolution = recMap.info.resolution
    origin_x = recMap.info.origin.position.x
    origin_y = recMap.info.origin.position.y
    
    # ----------------- Vypiš původní data z mapy jako hezkou tabulku -----------------
    print("----- Původní data z OccupancyGrid (1D) -----")
    data_list = list(recMap.data)
    row_length = 27  # počet hodnot na řádek
    
    for i in range(0, len(data_list), row_length):
        line = data_list[i:i+row_length]
        # vynecháme hranaté závorky, jen čísla zarovnaná na 3 znaky
        print(' '.join(f"{val:3}" for val in line))

    # 1D -> 2D numpy array ##################################################################################################
    map_np = np.array(recMap.data, dtype=np.int8).reshape((height, width))

    # Najdi volné a obsazené buňky
    free_cells = np.argwhere(map_np == 0)
    occupied_cells = np.argwhere(map_np == 100)

    height, width = map_np.shape

    # Převod všech bodů na světové souřadnice
    free_points_world = np.array([grid_to_world(r, c, origin_x, origin_y, resolution) for r, c in free_cells])
    occupied_points_world = np.array([grid_to_world(r, c, origin_x, origin_y, resolution) for r, c in occupied_cells])



    start = transform(2,0)
    #goal = transform(0,2)
    goal = transform(0,0)

    grid = (map_np == 100).astype(int)

    costmap = compute_costmap_8neighbors(grid)


    print(costmap)



    dist, parent = dijkstra_grid(grid, start, goal, costmap)
    path = reconstruct_path(parent, goal)
    
    print("Hrubá cesta:", path)
    
    # Line-of-sight smoothing
    path_smoothed = line_of_sight_smooth(path, costmap, threshold=0.11)
    print("Hladká cesta:", path_smoothed)




    #############################################################################################################################################

    

    # Převod hrubé cesty na světové souřadnice
    path_world = np.array([grid_to_world(r, c, origin_x, origin_y, resolution) for r, c in path])
    
    # Převod hladké cesty na světové souřadnice
    path_smoothed_world = np.array([grid_to_world(r, c, origin_x, origin_y, resolution) for r, c in path_smoothed])



    # Připrav souřadnice všech buněk
    ys, xs = np.indices(costmap.shape)
    
    points_world = np.array([
        grid_to_world(r, c, origin_x, origin_y, resolution)
        for r, c in zip(ys.flatten(), xs.flatten())
    ])

    # Cost hodnoty pouze pro FREE buňky
    free_cost_values = np.array([
        costmap[r, c] for r, c in free_cells
    ])

    
    # ----------------- Vizualizace -----------------
    plt.figure(figsize=(6, 6))
    
    # FREE buňky s costmapou (odstíny šedi)
    sc = plt.scatter(
        free_points_world[:, 0],
        free_points_world[:, 1],
        c=1.0 - free_cost_values,  # invertujeme: bezpečné = bílé
        cmap='gray',
        s=100,
        edgecolor='k',
        vmin=0.0,
        vmax=1.0,
        label='Free (costmap)'
    )



    
    # OCCUPIED buňky – čistě černé
    plt.scatter(
        occupied_points_world[:, 0],
        occupied_points_world[:, 1],
        color='black',
        s=100,
        label='Occupied'
    )

    # Hrubá cesta (Dijkstra)
    if len(path_world) > 0:
        plt.plot(path_world[:,0], path_world[:,1], color='red', linewidth=2, linestyle='--', label='Hrubá cesta')
    
    # Hladká cesta (Line-of-sight smoothed)
    if len(path_smoothed_world) > 0:
        plt.plot(path_smoothed_world[:,0], path_smoothed_world[:,1], color='cyan', linewidth=2, label='Hladká cesta')

    
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.title('Labyrinth map with costmap (grayscale)')
    plt.legend()
    plt.axis('equal')
    plt.grid(True)
    
    # Colorbar – čitelný, ne rušivý
    cbar = plt.colorbar(sc)
    cbar.set_label('Safety (white = safe, dark = risky)')
    
    plt.show()





















    









# resolution = 1 # může být také 0.66>, 0.33> nebo 0.166>
# startPos = (2,1)
# goalPos = (0,0) # - globální!
# robotWidth = ?
# secureWallDistance = ?

# vezmu počáteční bod
# checknu 8 okolních bodů a zjistím, který je nejblíže cíli
# - checknu, zda každý bod existuje
# - checknu, zda je každý bod dostupný
# ověřím, zda na ten bod mám přístup
# - pokud ne, jdu na druhý nejbližší
# - pokud ano, jdu na tento bod a čeknu opět všech 8 okolních bodů
# - opakování...
# - NEJKRATŠÍ CESTY ZKOUMÁM JAKO PRVNÍ
# - využiju rekurzi
# Použiju BFS - hledání nejkrajší cesty na úkor výkonové náročnosti.

#Edges have these properties:
# - Parent Node
# - Child Node
# - Cost (celou délku od počátečního bodu)










"""

✔️ For A* I need:

Nodes – every free cell of the grid
Edges – movement between neighboring cells
g(n) – cost from start to the current node (travelled distance, usually 1 or sqrt(2) when using diagonals)
h(n) – heuristic; estimate of how far node n is from the goal
    - import numpy as np
    - h = np.sqrt((x_current - x_goal)**2 + (y_current - y_goal)**2)
    - (diagonals allowed)
f(n) = g(n) + h(n) – total score of a node, used to select the next step

✔️ Implementation:

# g: distance from start
g_score[neighbor] = g_score[current] + cost(current, neighbor)

# h: heuristic to goal
h = np.linalg.norm(np.array(neighbor) - np.array(goal))

# f = g + h
f = g_score[neighbor] + h


g_score is a dictionary storing the actual cost from start to that node.

f is used for the priority queue (heapq) → the node with the smallest f is expanded first.

✔️ I have a problem here:

The starting position is (2,1).

I have four options to go: (2,0), (3,1), (2,2), (1,1).

g(n) = 0

h(n) according to
"h(n) = abs(x_current - x_goal) + abs(y_current - y_goal)"
comes out as 1 everywhere.

What now? I cannot determine an effective decision of where to go, right?

The result should be that the point (2,0) and (1,1) is the best option.

Solution: Tie-breaker

Prefer the direction toward the goal.

Add a small factor based on direction to the goal, e.g.:

f = g + h + eps * (dx + dy)

eps is a small number (0.001)

(dx, dy) → difference between neighbor and start/goal

dx = abs(neighbor[0] - goal[0])
dy = abs(neighbor[1] - goal[1])


This breaks the equality and A* prefers nodes “more directly toward the goal”.

The solution doesn't work. The values for all four neighbors are still equivalent.

The purpose of the tie-breaker is to decide between neighbors when they have the same f = g + h.
Therefore the tie-breaker must reflect the direction of movement, not just the distance to the goal.

Directional tie-breaker

You can compute the vector from the start to the neighbor and compare it to the direction from the start to the goal.

# direction vector from start to neighbor
move_vector = (neighbor[0]-start[0], neighbor[1]-start[1])

# direction vector from start to goal
goal_vector = (goal[0]-start[0], goal[1]-start[1])

# dot product
dot = move_vector[0]*goal_vector[0] + move_vector[1]*goal_vector[1]

# larger dot → movement “more directly” toward the goal
# tie-breaker f = f - eps * dot  (the larger the dot, the smaller the f)


✔️ Robot width:

How is robot size handled in practice?

Map inflation (most common solution)

In ROS this is handled by costmap_2d → obstacle inflation.

Principle:
Every occupied cell is “inflated” by the robot radius + safety margin.

Example: robot diameter 0.4 m, resolution 0.05 m:

robot_radius = 0.2 m
safety_margin = 0.05 m
inflate_radius = (0.2 + 0.05) / 0.05 = 5 cells


Result:
The map gets a “thicker” version of the walls → the robot plans wider corridors.

✔ simple
✔ works for global and local planning
✔ automatic in ROS
✘ enlarges the map and may eliminate very narrow passages

✔️ How to determine nodes requiring a decision?

Based on number of free neighbors.

For each free cell, count the number of free neighboring cells:

# 8-directional movement
directions = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]

def count_free_neighbors(grid, row, col):
    count = 0
    for dr, dc in directions:
        r, c = row + dr, col + dc
        if 0 <= r < grid.shape[0] and 0 <= c < grid.shape[1]:
            if grid[r, c] == 0:  # free
                count += 1
    return count

# decision node:
if count_free_neighbors(grid, row, col) > 2:
    is_decision_node = True


discoveredNodes = []

✔️ BFS pseudo code

def BFS(currentNode, graph, goal)
    Add currentNode to queue
    while queue not empty:
        n = queue.pop
        if n == goal: return
        for child in getChildren(graph, n):
            if child not in listParents(discoveredNodes):
                add new edge from n to child to discoveredNodes
                add child to queue


⭐ Line-of-sight smoothing

Problem: Moving in 8 directions is not enough – the path will still be inefficient.
Comparing every cell to every other one is inefficient.
But I don’t actually need to compare every cell to every other one.
First, Dijkstra gives me the “shortest” path to the goal – but it may have shortcomings because movement is limited to 8 directions.
But I can then refine it and remove unnecessary points and go straight instead.

Solution: Dijkstra or A* combined with Line-of-sight smoothing:

1️⃣ First you find a path on the grid and get a rough path like: A → x1 → x2 → x3 → x4 → B.
It is correct but “stair-like”, because the grid limits movement.

2️⃣ Then you smooth the path. You iterate through the points:

start at A

try to find the farthest point B visible with a “straight line” without collision

discard all intermediate points

continue from that point

def line_of_sight_smooth(path, can_see_func):
    result = [path[0]]
    i = 0

    while i < len(path) - 1:
        j = len(path) - 1

        # find the farthest visible point
        while j > i and not can_see_func(path[i], path[j]):
            j -= 1

        result.append(path[j])
        i = j

    return result

def bresenham_line(x1, y1, x2, y2):
    # Returns list of cells between two points using Bresenham's algorithm.
    points = []
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    x, y = x1, y1
    sx = 1 if x2 > x1 else -1
    sy = 1 if y2 > y1 else -1

    if dx >= dy:
        err = dx // 2
        while x != x2:
            points.append((x, y))
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy // 2
        while y != y2:
            points.append((x, y))
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy

    points.append((x2, y2))
    return points

def can_see_func(a, b, obstacles=set()):
    # Returns True if there is no obstacle between points a and b.
    x1, y1 = a
    x2, y2 = b

    for cell in bresenham_line(x1, y1, x2, y2):
        if cell in obstacles:
            return False

    return True


Result: A → x4 → B, where x4 is the first point reachable by a free straight line from A.
And you get a nice optimal path.

Good, so using Dijkstra without diagonals together with Line-of-sight smoothing seems like the best and cheapest solution.
I don’t need diagonals in Dijkstra because Line-of-sight smoothing will handle them afterward.
This saves performance.

⭐ Path offsetting

You have the smoothed path after line-of-sight smoothing: A → x1 → x2 → x3 → B

For each cell/point on the path:

Compute the nearest distance to an obstacle

If it's smaller than min_clearance:

push the point away from the obstacle (in the direction of the vector from the obstacle to the point)

the offset must be smaller than the distance to the existing path to keep the path smooth

Result: the path is smooth and always maintains a safe distance from obstacles.

(I will probably only handle clearance from corners)

CONVOLUTION

numpy unifie..

"""
























"""

✔️ Pro A* potřebuju:

Uzly – každá volná buňka gridu
Hrany – pohyb mezi sousedními buňkami
g(n) – náklad z startu k aktuálnímu uzlu (dosavadní vzdálenost, zpravidla 1 nebo sqrt(2) při použití diagonál)
h(n) – heuristika; odhad, jak daleko je uzel n od cíle
    - import numpy as np
    - h = np.sqrt((x_current - x_goal)**2 + (y_current - y_goal)**2)
    - (diagonály povoleny)
f(n) = g(n) + h(n) – celkové skóre uzlu, podle kterého vybíráme další krok

✔️ Implementace:

# g: distance from start
g_score[neighbor] = g_score[current] + cost(current, neighbor)

# h: heuristic to goal
h = np.linalg.norm(np.array(neighbor) - np.array(goal))

# f = g + h
f = g_score[neighbor] + h

- g_score je slovník, který ukládá skutečný náklad z startu k danému uzlu.
- f se používá pro priority queue (heapq) → uzel s nejmenším f se zkoumá jako první.

✔️ Mám tady problém:

Začáteční pozice je (2,1).

Mám čtyři možnosti, kam mohu jít: (2,0), (3,1), (2,2), (1,1).

g(n) = 0

h(n) podle "h(n) = abs(x_current - x_goal) + abs(y_current  - y_goal)" vyjde všude 1. 

Co teď? Nemohu určit efektivní rozhodnutí, kam jít, ne?

Výsledkem by mělo být, že bod (2,0) a (1,1) je nejlepší možnost, kam jít.

>>> Řešení: Tie-breaker

Upřednostnit směr k cíli

Přidej malý faktor podle směru k cíli, např.:

f = g + h + eps * (dx + dy)

eps malé číslo (0.001)

(dx, dy) → rozdíl mezi sousedem a startem/cílem

dx = abs(neighbor[0] - goal[0])
dy = abs(neighbor[1] - goal[1])

To rozbije rovnost a A* preferuje uzly „přímo k cíli“

>>> Řešení nefunguje. Hodnoty pro všechny čtyři sousedy jsou pořád ekvivalentní.

Cílem tie-breakeru je rozhodnout mezi sousedy, když mají stejné f = g + h.
Proto tie-breaker musí reflektovat směr pohybu, ne jen vzdálenost k cíli.

>>> Směrový tie-breaker

Můžeš spočítat vektor od startu k sousedovi a porovnat ho se směrem od startu k cíli.

# směrový vektor od startu k sousedovi
move_vector = (neighbor[0]-start[0], neighbor[1]-start[1])

# směrový vektor od startu k cíli
goal_vector = (goal[0]-start[0], goal[1]-start[1])

# dot product
dot = move_vector[0]*goal_vector[0] + move_vector[1]*goal_vector[1]

# větší dot → pohyb „více přímo“ k cíli
# tie-breaker f = f - eps * dot  (čím větší dot, tím menší f)

✔️ Šířka robota:

Jak se velikost robota řeší v praxi?

Inflace mapy (nejčastější řešení)

V ROSu to dělá costmap_2d → inflace překážek.

Princip:
Každá obsazená buňka se „nafoukne“ o poloměr robota + bezpečnostní vzdálenost.

Např. robot má průměr 0.4 m, resolution je 0.05 m:

robot_radius = 0.2 m
safety_margin = 0.05 m
inflate_radius = (0.2 + 0.05) / 0.05 = 5 buněk

Výsledek:
V mapě vznikne „tlustší“ verze stěn → robot plánuje mezi nimi širší uličky.

✔ jednoduché
✔ funguje pro global i local planning
✔ automatické v ROSu
✘ mapu „zvětší“ a některé úzké průchody úplně eliminuje

✔️ Jak určit nody vyžadující rozhodnutí?

Na základě počtu volných sousedů

Pro každou volnou buňku spočítej počet volných sousedních buněk:

# 8-směrový pohyb
directions = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]

def count_free_neighbors(grid, row, col):
    count = 0
    for dr, dc in directions:
        r, c = row + dr, col + dc
        if 0 <= r < grid.shape[0] and 0 <= c < grid.shape[1]:
            if grid[r, c] == 0:  # free
                count += 1
    return count

# rozhodující uzel:
if count_free_neighbors(grid, row, col) > 2:
    is_decision_node = True

discoveredNodes = []

✔️ BFS pseudo code

def BFS(currentNode, graph, goal)
    Add currentNode to queue
    while queue not empty:
        n = queue.pop
        if n == goal: return
        for child in getChildren(graph, n):
            if child not in listParents(discoveredNodes):
                add new edge from n to child to discoveredNodes
                add child to queue

⭐ Line-of-sight smoothing

Problém: Jít 8mi směry nestačí - cesta stále bude neefektivní. Řešit to skrze porovnávání každé buňky s každou další buňkou je neefektvní. Já ale vlastně nepotřebuju porovnávat každou buňku z každou jinou buňkou. Nejprve pomocí djikstra vytvořím "nejkratší" cestu k cíli - kde ale může dojít k nedostatkům v tom, že se mohu pohybovat pouze 8 směry. To ale poté mohu znovu upravit a vyhodit další buňky z cesty, které nepotřebuji a mohu jít místo toho přímo.

Řešení: Dijkstra nebo A* doplněn o Line-of-sight smoothing (vyhlazování cesty):

1️⃣ Nejprve najdeš cestu po gridu a dostaneš hrubou cestu typu: A → x1 → x2 → x3 → x4 → B. Je to správná, ale „schodovitá“ cesta, protože grid omezuje směry.
2️⃣ Potom tuto cestu vyhladíš. Procházíš postupně body cesty:
- začni bodem A
- snaž se najít nejvzdálenější bod B, na který vidíš „přímou čarou“ bez kolize
- všechny mezilehlé body zahodíš
- pokračuj od tohoto bodu dál.

def line_of_sight_smooth(path, can_see_func):
    result = [path[0]]
    i = 0

    while i < len(path) - 1:
        j = len(path) - 1

        # hledej nejvzdálenější bod, který je vidět
        while j > i and not can_see_func(path[i], path[j]):
            j -= 1

        result.append(path[j])
        i = j

    return result


def bresenham_line(x1, y1, x2, y2):
    #Vrátí seznam buněk mezi dvěma body pomocí Bresenhamova algoritmu.
    points = []
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    x, y = x1, y1
    sx = 1 if x2 > x1 else -1
    sy = 1 if y2 > y1 else -1

    if dx >= dy:
        err = dx // 2
        while x != x2:
            points.append((x, y))
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy // 2
        while y != y2:
            points.append((x, y))
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy

    points.append((x2, y2))
    return points


def can_see_func(a, b, obstacles=set()):
    #Vrací True, pokud mezi body a a b není překážka.
    x1, y1 = a
    x2, y2 = b

    for cell in bresenham_line(x1, y1, x2, y2):
        if cell in obstacles:
            return False

    return True




Výsledek: A → x4 → B, kde x4 je první bod, na který vede volná přímka z A. A dostaneš krásnou optimální cestu.

Dobře, takže vlastně se mi zdá použít djikstra bez diagonál spolu s Line-of-sight smoothing jako nejlepší a nejlevnější řešení. Nepotřebuji u dijkstra dělat diagonály, protože je potom udělá Line-of-sight smoothing. Tím šetřím výkon.

⭐ Path offsetting

1. Máš vyhlazenou cestu po line-of-sight smoothingu: A → x1 → x2 → x3 → B

2. Pro každou buňku/cíl na cestě:
- Spočítáš nejbližší vzdálenost od překážky
- Pokud je menší než min_clearance:
    - posuneš bod od překážky pryč (ve směru vektoru od překážky k bodu)
    - posun musí být menší než vzdálenost ke stávající cestě, aby cesta zůstala hladká
3. Výsledek: cesta je hladká a vždy dodržuje bezpečný odstup od překážek.

(budu pravděpodobně řešit pouze odstup od rohů)

"""

