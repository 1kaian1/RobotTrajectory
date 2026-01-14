#!/usr/bin/env python3

import rospy
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid
import numpy as np
import matplotlib.pyplot as plt
import heapq
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf.transformations as tf_trans

current_pose = None
goal_pose = None

def current_pose_cb(msg):
    global current_pose
    current_pose = msg.pose.pose #############################

def goal_pose_cb(msg):
    global goal_pose
    goal_pose = msg ##########################################

def world_to_grid(x, y, origin_x, origin_y, resolution):
    col = int((x - origin_x) / resolution)
    row = int((y - origin_y) / resolution)
    return (row, col)

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
        for move_row, move_col in [(1, 0),  (-1, 0),  (0, 1),  (0, -1)]:
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

def compute_orientations(path_world):
    """
    path_world: list of (x, y)
    returns: list of (x, y, theta)
    """
    path_with_theta = []

    for i in range(len(path_world)):
        x, y = path_world[i]

        if i < len(path_world) - 1:
            x2, y2 = path_world[i + 1]
            theta = math.atan2(y2 - y, x2 - x)
        else:
            # last pose → keep previous orientation
            theta = path_with_theta[-1][2] if path_with_theta else 0.0

        path_with_theta.append((x, y, theta))

    return path_with_theta

def publish_path(path_world_theta, frame_id="map"):
    path_msg = Path()
    path_msg.header.stamp = rospy.Time.now()
    path_msg.header.frame_id = frame_id

    for x, y, theta in path_world_theta:
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = frame_id

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        qx, qy, qz, qw = tf_trans.quaternion_from_euler(0, 0, theta)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        path_msg.poses.append(pose)

    path_pub.publish(path_msg)

def publish_costmap(costmap, recMap, frame_id="map"):
    grid_msg = OccupancyGrid()
    grid_msg.header.stamp = rospy.Time.now()
    grid_msg.header.frame_id = frame_id

    grid_msg.info.resolution = recMap.info.resolution
    grid_msg.info.width = recMap.info.width
    grid_msg.info.height = recMap.info.height
    grid_msg.info.origin = recMap.info.origin

    # Convert float [0.0–1.0] → int [0–100]
    costmap_scaled = (costmap * 100).astype(np.int8)

    # ROS uses 1D row-major
    grid_msg.data = costmap_scaled.flatten().tolist()

    costmap_pub.publish(grid_msg)


if __name__ == "__main__":
    rospy.init_node('moro_maze_navigation', anonymous=True)

    # --- Publishers ---
    path_pub = rospy.Publisher("/global_path", Path, queue_size=1, latch=True)
    costmap_pub = rospy.Publisher("/custom_costmap", OccupancyGrid, queue_size=1, latch=True)

    # --- Subscribers ---
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, goal_pose_cb)  # RViz 2D Nav Goal
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, current_pose_cb)

    rospy.loginfo("Getting map...")
    recMap = getMap()
    if recMap is None:
        rospy.logerr("Could not get map, exiting")
        exit(1)

    # --- Map setup ---
    width = recMap.info.width
    height = recMap.info.height
    resolution = recMap.info.resolution
    origin_x = recMap.info.origin.position.x
    origin_y = recMap.info.origin.position.y

    map_np = np.array(recMap.data, dtype=np.int8).reshape((height, width))
    grid = (map_np == 100).astype(int)

    # --- Compute and publish costmap once ---
    costmap = compute_costmap_8neighbors(grid)
    publish_costmap(costmap, recMap)
    rospy.loginfo("Custom costmap published")

    # --- Fixed start for testing ---
    #start_x, start_y = 0.0, 0.0  # Change if needed
    #start = world_to_grid(start_x, start_y, origin_x, origin_y, resolution)

    #start_x = current_pose.position.x
    #start_y = current_pose.position.y
    #start = world_to_grid(start_x, start_y, origin_x, origin_y, resolution)


    rospy.loginfo("Waiting for 2D Nav Goal from RViz...")
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        if current_pose is None or goal_pose is None: #################################### current_pose nemůže být none
            rate.sleep()
            continue

        #if goal_pose is None:
        #    rate.sleep()
        #    continue

        start_x = current_pose.position.x
        start_y = current_pose.position.y
        start = world_to_grid(start_x, start_y, origin_x, origin_y, resolution)

        # --- Transform goal_pose to grid coordinates ---
        goal_x = goal_pose.pose.position.x
        goal_y = goal_pose.pose.position.y
        goal = world_to_grid(goal_x, goal_y, origin_x, origin_y, resolution)

        rospy.loginfo("Planning path from (%f,%f) to (%f,%f)", start_x, start_y, goal_x, goal_y)

        # --- Compute path ---
        dist, parent = dijkstra_grid(grid, start, goal, costmap)
        path = reconstruct_path(parent, goal)

        # --- Smooth path ---
        path_smoothed = line_of_sight_smooth(path, costmap, threshold=0.124)

        # --- Convert to world coordinates ---
        path_world = [grid_to_world(r, c, origin_x, origin_y, resolution) for r, c in path_smoothed]

        # --- Compute orientations ---
        path_with_theta = compute_orientations(path_world)

        # --- Print path and rotations ---
        print("global_path = [")
        for x, y, theta in path_with_theta:
            print(f"    [{x:.3f}, {y:.3f}, {theta:.3f}],")
        print("]\n")

        # --- Publish path ---
        publish_path(path_with_theta)
        rospy.loginfo("Global path published")

        # Reset goal to wait for next RViz input
        goal_pose = None
        rate.sleep()