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