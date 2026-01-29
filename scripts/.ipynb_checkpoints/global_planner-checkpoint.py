#!/usr/bin/env python3

import rospy
import math
import numpy as np
import tf.transformations as tf_trans
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

# =========================
# Global state
# =========================

current_pose = None
goal_pose = None

import heapq

search_pub = None  # publisher pro vizualizaci Dijkstry

# =========================
# Callbacks
# =========================

def current_pose_cb(msg):
    global current_pose
    # Store current robot pose from AMCL
    current_pose = msg.pose.pose

def goal_pose_cb(msg):
    global goal_pose
    # Store 2D Nav Goal from RViz
    goal_pose = msg

# =========================
# Coordinate transforms
# =========================

def world_to_grid(x, y, origin_x, origin_y, resolution):
    # Convert world coordinates to grid indices
    col = int((x - origin_x) / resolution)
    row = int((y - origin_y) / resolution)
    return row, col

def grid_to_world(row, col, origin_x, origin_y, resolution):
    # Convert grid cell indices to world coordinates (cell center)
    x = origin_x + (col + 0.5) * resolution
    y = origin_y + (row + 0.5) * resolution
    return x, y

# =========================
# Map handling
# =========================

def get_map():
    # Request static map from map server
    rospy.wait_for_service('static_map')
    try:
        get_map_srv = rospy.ServiceProxy('static_map', GetMap)
        return get_map_srv().map
    except rospy.ServiceException as e:
        rospy.logerr("Map service call failed: %s", e)
        return None

def compute_costmap_8neighbors(binary_grid):
    # For each cell, compute average occupancy of 8 neighbors
    # Result is a soft costmap in range 0.0–1.0
    h, w = binary_grid.shape
    costmap = np.zeros((h, w), dtype=np.float32)
    for r in range(h):
        for c in range(w):
            neighbor_sum = 0
            neighbor_count = 0
            for dr in [-1, 0, 1]:
                for dc in [-1, 0, 1]:
                    if dr == 0 and dc == 0:
                        continue
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < h and 0 <= nc < w:
                        neighbor_sum += binary_grid[nr, nc]
                        neighbor_count += 1
            # Normalize by number of valid neighbors
            costmap[r, c] = neighbor_sum / neighbor_count if neighbor_count > 0 else 0.0
    return costmap

# =========================
# Vizualizace Dijkstry
# =========================

def publish_search(search_grid, rec_map):
    msg = OccupancyGrid()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    msg.info = rec_map.info
    msg.data = search_grid.flatten().tolist()
    search_pub.publish(msg)

def dijkstra_visual(grid, start, goal, costmap, rec_map, alpha=10.0):
    h, w = grid.shape
    dist = {start: 0}
    prev = {start: None}
    pq = [(0, start)]
    search_vis = np.zeros((h, w), dtype=np.int8)

    while pq and not rospy.is_shutdown():
        d, (r, c) = heapq.heappop(pq)
        if (r, c) == goal:
            break
        if d > dist[(r, c)]:
            continue

        search_vis[r, c] = 100
        publish_search(search_vis, rec_map)
        rospy.sleep(0.01)  # zpomalení pro vizualizaci

        for dr, dc in [(1,0), (-1,0), (0,1), (0,-1)]:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < h and 0 <= nc < w):
                continue
            if grid[nr, nc] == 1:
                continue

            nd = d + 1 + alpha * costmap[nr, nc]
            nb = (nr, nc)
            if nb not in dist or nd < dist[nb]:
                dist[nb] = nd
                prev[nb] = (r, c)
                heapq.heappush(pq, (nd, nb))
                search_vis[nr, nc] = 50

    return prev

# =========================
# Path planning (Dijkstra)
# =========================

def dijkstra_grid(grid, start_cell, goal_cell, costmap, alpha=10.0):
    # Dijkstra search on grid with additional cost from costmap
    rows, cols = grid.shape
    distance = {start_cell: 0}
    previous = {start_cell: None}
    queue = [(0, start_cell)]
    while queue:
        current_dist, current = heapq.heappop(queue)
        if current == goal_cell:
            break
        if current_dist > distance[current]:
            continue
        r, c = current
        for dr, dc in [(1,0), (-1,0), (0,1), (0,-1)]:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < rows and 0 <= nc < cols):
                continue
            if grid[nr, nc] == 1:
                continue
            move_cost = 1
            cost = move_cost + alpha * costmap[nr, nc]
            new_dist = current_dist + cost
            neighbor = (nr, nc)
            if neighbor not in distance or new_dist < distance[neighbor]:
                distance[neighbor] = new_dist
                previous[neighbor] = current
                heapq.heappush(queue, (new_dist, neighbor))
    return distance, previous

def reconstruct_path(previous, goal_cell):
    # Reconstruct path by walking backwards from goal
    path = []
    cell = goal_cell
    while cell is not None:
        path.append(cell)
        cell = previous.get(cell)
    return path[::-1]

# =========================
# Line-of-sight smoothing
# =========================

def bresenham_line(x1, y1, x2, y2):
    # Generate grid cells along a line using Bresenham algorithm
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
    # Check if all cells between a and b are below cost threshold
    for r, c in bresenham_line(a[0], a[1], b[0], b[1]):
        if costmap[r, c] > threshold:
            return False
    return True

def line_of_sight_smooth(path, costmap, threshold=0.133):
    # Remove unnecessary intermediate points using line-of-sight check
    if len(path) <= 2:
        return path.copy()
    smoothed = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i and not can_see(path[i], path[j], costmap, threshold):
            j -= 1
        smoothed.append(path[j])
        i = j
    return smoothed

# =========================
# Orientation processing
# =========================

def compute_orientations(path_world):
    # Assign heading based on direction to next waypoint
    result = []
    for i in range(len(path_world)):
        x, y = path_world[i]
        if i < len(path_world) - 1:
            x2, y2 = path_world[i + 1]
            theta = math.atan2(y2 - y, x2 - x)
        else:
            # Keep previous orientation for last point
            theta = result[-1][2] if result else 0.0
        result.append((x, y, theta))
    return result

def shift_theta_forward_with_dup(path):
    # Duplicate waypoints to enforce rotation in place
    if not path:
        return []
    new_path = []
    prev_theta = 0.0
    for i, (x, y, theta) in enumerate(path):
        if i != 0:
            new_path.append([x, y, prev_theta])
        new_path.append([x, y, theta])
        prev_theta = theta
    return new_path

# =========================
# Publishing
# =========================

def publish_path(path_world_theta, frame_id="map"):
    # Publish path as nav_msgs/Path
    msg = Path()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id
    for x, y, theta in path_world_theta:
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose.position.x = x
        ps.pose.position.y = y
        qx, qy, qz, qw = tf_trans.quaternion_from_euler(0, 0, theta)
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        msg.poses.append(ps)
    path_pub.publish(msg)

def publish_costmap(costmap, rec_map, frame_id="map"):
    # Publish costmap as OccupancyGrid for visualization
    grid_msg = OccupancyGrid()
    grid_msg.header.stamp = rospy.Time.now()
    grid_msg.header.frame_id = frame_id
    grid_msg.info = rec_map.info
    grid_msg.data = (costmap * 100).astype(np.int8).flatten().tolist()
    costmap_pub.publish(grid_msg)

def print_path(path_theta, title):
    print(f"\n{title}")
    print("-" * 60)
    print(f"{'i':>3} | {'x':>8} | {'y':>8} | {'theta (rad)':>12}")
    print("-" * 60)
    for i, (x, y, theta) in enumerate(path_theta):
        print(f"{i:3d} | {x:8.3f} | {y:8.3f} | {theta:12.3f}")
    print("-" * 60)

# =========================
# Main
# =========================

if __name__ == "__main__":
    rospy.init_node("moro_maze_navigation")

    path_pub = rospy.Publisher("/global_path", Path, queue_size=1, latch=True)
    costmap_pub = rospy.Publisher("/custom_costmap", OccupancyGrid, queue_size=1, latch=True)
    search_pub = rospy.Publisher("/dijkstra_search", OccupancyGrid, queue_size=1, latch=True)

    rospy.Subscriber("/move_base_simple/goal", PoseStamped, goal_pose_cb)
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, current_pose_cb)

    rospy.loginfo("Loading map...")
    rec_map = get_map()
    if rec_map is None:
        rospy.logerr("Map not available")
        exit(1)

    w = rec_map.info.width
    h = rec_map.info.height
    res = rec_map.info.resolution
    ox = rec_map.info.origin.position.x
    oy = rec_map.info.origin.position.y

    raw = np.array(rec_map.data, dtype=np.int8).reshape((h, w))
    grid = (raw == 100).astype(int)

    costmap = compute_costmap_8neighbors(grid)
    publish_costmap(costmap, rec_map)
    rospy.loginfo("Costmap published")

    rate = rospy.Rate(10)
    rospy.loginfo("Waiting for pose and RViz goal...")

    while not rospy.is_shutdown():
        if current_pose is None or goal_pose is None:
            rate.sleep()
            continue

        sx = current_pose.position.x
        sy = current_pose.position.y
        gx = goal_pose.pose.position.x
        gy = goal_pose.pose.position.y

        start = world_to_grid(sx, sy, ox, oy, res)
        goal = world_to_grid(gx, gy, ox, oy, res)

        rospy.loginfo("Planning from (%.2f, %.2f) to (%.2f, %.2f)", sx, sy, gx, gy)

        # <-- použití vizualizace Dijkstry místo klasického dijkstra_grid -->
        parent = dijkstra_visual(grid, start, goal, costmap, rec_map)

        path = reconstruct_path(parent, goal)
        path = line_of_sight_smooth(path, costmap, threshold=0.124)
        path_world = [grid_to_world(r, c, ox, oy, res) for r, c in path]
        path_theta = compute_orientations(path_world)
        print_path(path_theta, "Global path BEFORE shift_theta_forward_with_dup")
        path_theta = shift_theta_forward_with_dup(path_theta)
        print_path(path_theta, "Global path AFTER shift_theta_forward_with_dup")

        publish_path(path_theta)
        rospy.loginfo("Global path published")

        goal_pose = None
        rate.sleep()
