#!/usr/bin/env python3
"""
global_planner.py

Global planner for TurtleBot3 navigation in ROS.

This module provides utilities for:
- Reading the map from the map server
- Converting between world and grid coordinates
- Computing costmaps based on occupancy
- Dijkstra search and path reconstruction
- Line-of-sight path smoothing
- Computing path orientations
- Publishing paths and costmaps to ROS topics
"""

import rospy
import math
import numpy as np
import tf.transformations as tf_trans
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import heapq

# =========================
# Global state
# =========================

current_pose = None
"""Current robot pose (from AMCL)."""

goal_pose = None
"""Current goal pose (from RViz 2D Nav Goal)."""

search_pub = None
"""Publisher for visualizing Dijkstra search progress."""

path_pub = None
"""Publisher for global path as nav_msgs/Path."""

costmap_pub = None
"""Publisher for custom costmap as OccupancyGrid."""


# =========================
# Callbacks
# =========================

def current_pose_cb(msg):
    """
    Callback for AMCL pose updates.

    Parameters
    ----------
    msg : PoseWithCovarianceStamped
        Current pose of the robot with covariance.
    """
    global current_pose
    current_pose = msg.pose.pose


def goal_pose_cb(msg):
    """
    Callback for 2D navigation goals from RViz.

    Parameters
    ----------
    msg : PoseStamped
        Goal position for the robot.
    """
    global goal_pose
    goal_pose = msg


# =========================
# Coordinate transforms
# =========================

def world_to_grid(x, y, origin_x, origin_y, resolution):
    """
    Convert world coordinates to grid indices.

    Parameters
    ----------
    x, y : float
        World coordinates
    origin_x, origin_y : float
        Map origin in world frame
    resolution : float
        Map resolution (meters per cell)

    Returns
    -------
    row, col : int
        Grid indices corresponding to world coordinates
    """
    col = int((x - origin_x) / resolution)
    row = int((y - origin_y) / resolution)
    return row, col


def grid_to_world(row, col, origin_x, origin_y, resolution):
    """
    Convert grid indices to world coordinates (cell center).

    Parameters
    ----------
    row, col : int
        Grid indices
    origin_x, origin_y : float
        Map origin in world frame
    resolution : float
        Map resolution (meters per cell)

    Returns
    -------
    x, y : float
        World coordinates
    """
    x = origin_x + (col + 0.5) * resolution
    y = origin_y + (row + 0.5) * resolution
    return x, y


# =========================
# Map handling
# =========================

def get_map():
    """
    Request the static map from the ROS map server.

    Returns
    -------
    OccupancyGrid or None
        Map retrieved from the server, or None on failure.
    """
    rospy.wait_for_service('static_map')
    try:
        get_map_srv = rospy.ServiceProxy('static_map', GetMap)
        return get_map_srv().map
    except rospy.ServiceException as e:
        rospy.logerr("Map service call failed: %s", e)
        return None


def compute_costmap_8neighbors(binary_grid):
    """
    Compute a soft costmap based on the 8-neighbor average.

    Parameters
    ----------
    binary_grid : np.ndarray
        Occupancy grid (0=free, 1=occupied)

    Returns
    -------
    costmap : np.ndarray
        Soft costmap with values in [0.0, 1.0]
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
                        continue
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < h and 0 <= nc < w:
                        neighbor_sum += binary_grid[nr, nc]
                        neighbor_count += 1
            costmap[r, c] = neighbor_sum / neighbor_count if neighbor_count > 0 else 0.0
    return costmap


# =========================
# Visualization
# =========================

def publish_search(search_grid, rec_map):
    """
    Publish Dijkstra search visualization as OccupancyGrid.

    Parameters
    ----------
    search_grid : np.ndarray
        Grid showing search progress
    rec_map : OccupancyGrid
        Reference map for header/info
    """
    msg = OccupancyGrid()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    msg.info = rec_map.info
    msg.data = search_grid.flatten().tolist()
    search_pub.publish(msg)


def dijkstra_visual(grid, start, goal, costmap, rec_map, alpha=10.0):
    """
    Dijkstra search with real-time visualization.

    Parameters
    ----------
    grid : np.ndarray
        Binary occupancy grid
    start : tuple[int, int]
        Start cell (row, col)
    goal : tuple[int, int]
        Goal cell (row, col)
    costmap : np.ndarray
        Soft costmap
    rec_map : OccupancyGrid
        Map reference for publishing
    alpha : float
        Costmap weight

    Returns
    -------
    prev : dict
        Mapping of each cell to its parent for path reconstruction.
    """
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
        rospy.sleep(0.01)

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


def reconstruct_path(previous, goal_cell):
    """
    Reconstruct path from parent mapping.

    Parameters
    ----------
    previous : dict
        Parent mapping from Dijkstra
    goal_cell : tuple[int, int]
        Goal cell

    Returns
    -------
    path : list[tuple[int, int]]
        Ordered path from start to goal
    """
    path = []
    cell = goal_cell
    while cell is not None:
        path.append(cell)
        cell = previous.get(cell)
    return path[::-1]


def bresenham_line(x1, y1, x2, y2):
    """
    Bresenham line algorithm for grid cells.

    Parameters
    ----------
    x1, y1 : int
        Start cell
    x2, y2 : int
        End cell

    Returns
    -------
    points : list[tuple[int,int]]
        List of cells along the line
    """
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
    Check line-of-sight between two cells using costmap threshold.

    Parameters
    ----------
    a, b : tuple[int,int]
        Grid cells
    costmap : np.ndarray
        Soft costmap
    threshold : float
        Max allowed cost for line-of-sight

    Returns
    -------
    visible : bool
        True if line-of-sight is clear
    """
    for r, c in bresenham_line(a[0], a[1], b[0], b[1]):
        if costmap[r, c] > threshold:
            return False
    return True


def line_of_sight_smooth(path, costmap, threshold=0.133):
    """
    Smooth path by removing unnecessary intermediate points.

    Parameters
    ----------
    path : list[tuple[int,int]]
        Original path
    costmap : np.ndarray
        Soft costmap
    threshold : float
        Max allowed cost for line-of-sight

    Returns
    -------
    smoothed : list[tuple[int,int]]
        Smoothed path
    """
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


def compute_orientations(path_world):
    """
    Compute orientations (theta) along path.

    Parameters
    ----------
    path_world : list[tuple[float,float]]
        Path in world coordinates

    Returns
    -------
    path_theta : list[tuple[float,float,float]]
        Path with (x,y,theta)
    """
    result = []
    for i in range(len(path_world)):
        x, y = path_world[i]
        if i < len(path_world) - 1:
            x2, y2 = path_world[i + 1]
            theta = math.atan2(y2 - y, x2 - x)
        else:
            theta = result[-1][2] if result else 0.0
        result.append((x, y, theta))
    return result


def shift_theta_forward_with_dup(path):
    """
    Duplicate waypoints to enforce rotation in place.

    Parameters
    ----------
    path : list[tuple[float,float,float]]
        Original path

    Returns
    -------
    new_path : list[list[float,float,float]]
        Path with duplicated points for orientation adjustment
    """
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


def publish_path(path_world_theta, frame_id="map"):
    """
    Publish path as nav_msgs/Path to ROS.

    Parameters
    ----------
    path_world_theta : list[tuple[float,float,float]]
        Path with orientation
    frame_id : str
        ROS frame id
    """
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
    """
    Publish soft costmap as OccupancyGrid to ROS.

    Parameters
    ----------
    costmap : np.ndarray
        Soft costmap
    rec_map : OccupancyGrid
        Reference map
    frame_id : str
        ROS frame id
    """
    grid_msg = OccupancyGrid()
    grid_msg.header.stamp = rospy.Time.now()
    grid_msg.header.frame_id = frame_id
    grid_msg.info = rec_map.info
    grid_msg.data = (costmap * 100).astype(np.int8).flatten().tolist()
    costmap_pub.publish(grid_msg)


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

        parent = dijkstra_visual(grid, start, goal, costmap, rec_map)
        path = reconstruct_path(parent, goal)
        path = line_of_sight_smooth(path, costmap, threshold=0.124)
        path_world = [grid_to_world(r, c, ox, oy, res) for r, c in path]
        path_theta = compute_orientations(path_world)
        path_theta = shift_theta_forward_with_dup(path_theta)

        publish_path(path_theta)
        rospy.loginfo("Global path published")

        goal_pose = None
        rate.sleep()
