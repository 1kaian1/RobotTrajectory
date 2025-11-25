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

    # 1D -> 2D numpy array
    map_np = np.array(recMap.data, dtype=np.int8).reshape((height, width))

    # Najdi volné a obsazené buňky
    free_cells = np.argwhere(map_np == 0)
    occupied_cells = np.argwhere(map_np == 100)

    # Převod všech bodů na světové souřadnice
    free_points_world = np.array([grid_to_world(r, c, origin_x, origin_y, resolution) for r, c in free_cells])
    occupied_points_world = np.array([grid_to_world(r, c, origin_x, origin_y, resolution) for r, c in occupied_cells])

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
goalPos = (0,0) # - globáln!
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

import math

directions = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]

def explore(node_x, node_y, grid):
    
    for move_x, move_y in directions:
        neighbor_x, neighbor_y = node_x + move_x, node_y + move_y
        path_lenght = math.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)




def euclidean_distance(a, b):
    return math.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)

# příklad
a = (1, 2)
b = (4, 6)
print(euclidean_distance(a, b))  # 5.0








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

"""

