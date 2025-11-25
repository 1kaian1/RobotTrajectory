#!/usr/bin/env python3

import rospy
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid
import numpy as np
import matplotlib.pyplot as plt

# ----------------- Helper Methods -----------------
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

# ----------------- Main -----------------
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



    # 1D -> 2D numpy array
    map_np = np.array(recMap.data, dtype=np.int8).reshape((height, width))

    # Najdi volné a obsazené buňky
    free_cells = np.argwhere(map_np == 0)
    occupied_cells = np.argwhere(map_np == 100)
    height, width = map_np.shape
    grid = np.array([[r, c] for r in range(height) for c in range(width)]) #######################################################################

    # Převod všech bodů na světové souřadnice
    free_points_world = np.array([grid_to_world(r, c, origin_x, origin_y, resolution) for r, c in free_cells])
    occupied_points_world = np.array([grid_to_world(r, c, origin_x, origin_y, resolution) for r, c in occupied_cells])

    # ----------------- Vypisování do konzole -----------------
    print("----- Volné buňky -----")
    for (r, c), (x, y) in zip(free_cells, free_points_world):
        print(f"Grid: ({r}, {c}) → World: ({x:.3f}, {y:.3f})")

    print("\n----- Obsazené buňky -----")
    for (r, c), (x, y) in zip(occupied_cells, occupied_points_world):
        print(f"Grid: ({r}, {c}) → World: ({x:.3f}, {y:.3f})")

    # ----------------- Vizualizace -----------------
    plt.figure(figsize=(6,6))
    plt.scatter(free_points_world[:,0], free_points_world[:,1], color='white', s=100, edgecolor='k', label='Free')
    plt.scatter(occupied_points_world[:,0], occupied_points_world[:,1], color='black', s=100, label='Occupied')
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.title('Labyrinth map')
    plt.legend()
    plt.axis('equal')
    plt.grid(True)
    plt.show()



# resolution = 1 # může být také 0.66>, 0.33> nebo 0.166>
# startPos = (2,1)
# goalPos = (0,0) # - globální!
# robotWidth = ?
# secureWallDistance = ?

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

import math

resolution = 1 # doplnit, navázat!
goal_pos = (0,0) # - globální!
directions = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]
distances = []

def explore_neighbors(node_coor, grid):
    
    for move_x, move_y in directions:
        neighbor_x, neighbor_y = node_coor[0] + move_x, node_coor[1] + move_y
        # if (neighbor_x, neighbor_y) in grid and check_for_wall_collision() == False:
            path_lenght = math.sqrt((neighbor_x - goal_pos[0])**2 + (neighbor_y - goal_pos[1])**2)
            distances.append(path_lenght)
        # else:
            distances.append(100)

    # dictionary pro seznam objevených node, key budou souřadnice node, hodnoty budou cost, parent a child
    # budu muset implementovat depth u nodes

    new_pos = (node_x, node_y) + directions[distances.index(min(distances))]

    if new_pos = goal_pos:
        pass

    
    explore_neighbors(new_pos, grid)

    # - stěny
    # - cíl dosažen
    # - pouze delší cesty

    # struktury
    # - nody se vzdálenostmi sousedů od cíle

def check_for_wall_collision():

    



def BFS(currentNode, graph, goal)
    Add currentNode to queue
    while queue not empty:
        n = queue.pop
        if n == goal: return
        for child in getChildren(graph, n):
            if child not in listParents(discoveredNodes):
                add new edge from n to child to discoveredNodes
                add child to queue


import heapq
import math
import numpy as np

# pohyb ve 8 směrech
MOVES = [
    (1, 0),  (-1, 0),  (0, 1),  (0, -1),      # ortogonální
    (1, 1),  (1, -1), (-1, 1), (-1, -1)       # diagonální
]

def dijkstra_grid(grid, start_cell, goal_cell):
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
            if grid[neighbor_row, neighbor_col] == 1: #######################################################################################
                continue

            # realistická cena pohybu:
            # diagonála = sqrt(2), rovně = 1
            if move_row != 0 and move_col != 0:
                move_cost = math.sqrt(2)
            else:
                move_cost = 1

            new_distance = current_distance + move_cost

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




grid = np.array([
    [0,0,0],
    [1,1,0],
    [0,0,0]
])

start = (0,0)
goal = (2,2)

dist, parent = dijkstra_grid_diagonal(grid, start, goal)
path = reconstruct_path(parent, goal)

print("Vzdálenost:", dist[goal])
print("Cesta:", path)






# Se secureWallDistance si stále nevím rady.



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

