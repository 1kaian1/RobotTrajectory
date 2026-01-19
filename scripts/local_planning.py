#!/usr/bin/env python3

import numpy as np
import numpy.typing as npt
import math
import rospy
import tf2_ros
import copy
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import Twist, PoseStamped, Pose, Point
from nav_msgs.msg import Path
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

# -------------------------------------------------------------------
# Global state
# -------------------------------------------------------------------

global_path = None
current_goal_ID = 0
path_received = False
final_goal_reached = False

# Goal reaching thresholds
GOAL_DISTANCE_THRESHOLD = 0.15   # meters
GOAL_ANGLE_THRESHOLD = 0.02      # radians

# -------------------------------------------------------------------
# Global path callback
# -------------------------------------------------------------------

def global_path_callback(msg: Path):
    """
    Receives global path and converts it to list of [x, y, theta].
    Resets goal index.
    """
    global global_path, path_received, current_goal_ID, final_goal_reached

    global_path = []
    for pose_stamped in msg.poses:
        x = pose_stamped.pose.position.x
        y = pose_stamped.pose.position.y
        q = pose_stamped.pose.orientation
        theta = R.from_quat([q.x, q.y, q.z, q.w]).as_euler("xyz")[2]
        global_path.append([x, y, theta])

    path_received = True
    current_goal_ID = 0
    final_goal_reached = False

    print(f"Global path received with {len(global_path)} waypoints")

# -------------------------------------------------------------------
# ROS init
# -------------------------------------------------------------------

rospy.init_node("local_planner")

path_sub = rospy.Subscriber("/global_path", Path, global_path_callback)

tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

cmd_pub  = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
traj_pub = rospy.Publisher("/local_plan", Path, queue_size=10)
goal_pub = rospy.Publisher("/local_goal", PoseStamped, queue_size=10)
traj_viz_pub = rospy.Publisher("/all_local_trajectories", MarkerArray, queue_size=1)

# -------------------------------------------------------------------
# Utility functions
# -------------------------------------------------------------------

def wrap_angle(theta: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return (theta + np.pi) % (2*np.pi) - np.pi


def localiseRobot():
    """
    Returns robot pose in map frame as [x, y, theta].
    Returns None if TF is not available.
    """
    try:
        trans = tfBuffer.lookup_transform(
            "map", "base_link", rospy.Time(0), rospy.Duration(0.2)
        )
    except:
        return None

    q = trans.transform.rotation
    theta = R.from_quat([q.x, q.y, q.z, q.w]).as_euler("xyz")[2]

    return np.array([
        trans.transform.translation.x,
        trans.transform.translation.y,
        theta
    ])


def transform_goal_to_robot_frame(robot_pose, goal_pose):
    """
    Transform goal from map frame to robot frame.
    """
    dx = goal_pose[0] - robot_pose[0]
    dy = goal_pose[1] - robot_pose[1]
    dtheta = goal_pose[2] - robot_pose[2]

    c = np.cos(-robot_pose[2])
    s = np.sin(-robot_pose[2])

    x_r = c*dx - s*dy
    y_r = s*dx + c*dy
    th_r = wrap_angle(dtheta)

    return np.array([x_r, y_r, th_r])


def check_goal_reached(robot_pose, goal_pose):
    """
    Checks if robot is close enough to goal (position + orientation).
    """
    dist = np.linalg.norm(robot_pose[:2] - goal_pose[:2])
    ang  = abs(wrap_angle(robot_pose[2] - goal_pose[2]))
    return dist <= GOAL_DISTANCE_THRESHOLD and ang <= GOAL_ANGLE_THRESHOLD


# -------------------------------------------------------------------
# Control generation
# -------------------------------------------------------------------

def generateControls(lastControl: npt.ArrayLike) -> np.ndarray:
    """
    Generates candidate (v, w) controls around previous command.
    """
    v_last, w_last = lastControl

    # Limits
    v_min, v_max = -0.2, 0.2
    w_min, w_max = -0.8, 0.8

    # Max change per step
    v_delta_max = 0.1
    w_delta_max = 0.8

    v_low  = max(v_min, v_last - v_delta_max)
    v_high = min(v_max, v_last + v_delta_max)
    w_low  = max(w_min, w_last - w_delta_max)
    w_high = min(w_max, w_last + w_delta_max)

    v_values = np.linspace(v_low, v_high, 10)
    w_values = np.linspace(w_low, w_high, 25)

    return np.array([[v, w] for v in v_values for w in w_values])


# -------------------------------------------------------------------
# Robot model
# -------------------------------------------------------------------

def forwardKinematics(control, lastPose, dt):
    """
    Differential drive forward kinematics.
    Pose: [x, y, theta]
    Control: [v, w]
    """
    vt, wt = control
    if abs(wt) < 1e-6:
        wt = 1e-6

    x, y, theta = lastPose
    vtwt = vt / wt

    return np.array([
        x - vtwt*np.sin(theta) + vtwt*np.sin(theta + wt*dt),
        y + vtwt*np.cos(theta) - vtwt*np.cos(theta + wt*dt),
        theta + wt*dt
    ])


class PT2Block:
    """
    Simple second-order dynamics model (Tustin approx).
    Used to smooth velocity.
    """
    def __init__(self, T=0, D=0, kp=1, ts=0, bufferLength=3):
        self.e = [0]*bufferLength
        self.y = [0]*bufferLength
        if ts != 0:
            self.setConstants(T, D, kp, ts)

    def setConstants(self, T, D, kp, ts):
        self.k1 = 4*T**2 + 4*D*T*ts + ts**2
        self.k2 = 2*ts**2 - 8*T**2
        self.k3 = 4*T**2 - 4*D*T*ts + ts**2
        self.k4 = kp*ts**2
        self.k5 = 2*kp*ts**2
        self.k6 = kp*ts**2

    def update(self, e):
        self.e = [e] + self.e[:-1]
        self.y = [0] + self.y[:-1]
        e, y = self.e, self.y
        y[0] = (e[0]*self.k4 + e[1]*self.k5 + e[2]*self.k6
                - y[1]*self.k2 - y[2]*self.k3) / self.k1
        return y[0]


# -------------------------------------------------------------------
# Cost function
# -------------------------------------------------------------------

def costFn(pose, goalpose, control):
    """
    Quadratic cost in state and control.
    """
    e = pose - goalpose
    e[2] = wrap_angle(e[2])

    Q = np.diag([1.0, 1.0, 0.1])
    Rm = np.diag([0.1, 0.5])

    return float(e.T @ Q @ e + control.T @ Rm @ control)


def evaluateControls(controls, robotModelPT2, horizon, goalpose_local):
    """
    Simulates all controls forward and evaluates cost.
    """
    costs = np.zeros(len(controls))
    trajectories = [[] for _ in controls]

    for i, control in enumerate(controls):
        sim_model = copy.deepcopy(robotModelPT2)
        pose = np.array([0.0, 0.0, 0.0])

        for _ in range(horizon):
            v, w = control
            v_dyn = sim_model.update(v)
            pose = forwardKinematics([v_dyn, w], pose, ts)
            costs[i] += costFn(pose, goalpose_local, control)
            trajectories[i].append(pose.copy())

    return costs, trajectories


# -------------------------------------------------------------------
# Visualization
# -------------------------------------------------------------------

def publish_all_trajectories(trajectories, costs):
    """
    Publishes all candidate trajectories colored by cost.
    """
    ma = MarkerArray()
    min_c, max_c = min(costs), max(costs)
    rng = max_c - min_c if max_c != min_c else 1.0

    for i, traj in enumerate(trajectories):
        m = Marker()
        m.header.frame_id = "base_link"
        m.header.stamp = rospy.Time.now()
        m.ns = "local_trajectories"
        m.id = i
        m.type = Marker.LINE_STRIP
        m.scale.x = 0.01

        norm = (costs[i] - min_c) / rng
        m.color.r = norm
        m.color.g = 1.0 - norm
        m.color.b = 0.0
        m.color.a = 0.6

        for p in traj:
            pt = Point(p[0], p[1], 0.02)
            m.points.append(pt)

        ma.markers.append(m)

    traj_viz_pub.publish(ma)


def publish_best_trajectory(traj):
    """
    Publishes the best trajectory in blue.
    """
    m = Marker()
    m.header.frame_id = "base_link"
    m.header.stamp = rospy.Time.now()
    m.ns = "best_trajectory"
    m.id = 0
    m.type = Marker.LINE_STRIP
    m.scale.x = 0.03
    m.color.b = 1.0
    m.color.a = 1.0

    for p in traj:
        m.points.append(Point(p[0], p[1], 0.03))

    traj_viz_pub.publish(MarkerArray(markers=[m]))


# -------------------------------------------------------------------
# Publishers
# -------------------------------------------------------------------

def pubCMD(control):
    v, w = control
    msg = Twist()
    msg.linear.x = float(v)
    msg.angular.z = float(w)
    cmd_pub.publish(msg)


def pubTrajectory(trajectory):
    path = Path()
    path.header.stamp = rospy.Time.now()
    path.header.frame_id = "base_link"

    for x, y, th in trajectory:
        ps = PoseStamped()
        ps.header = path.header
        ps.pose.position.x = x
        ps.pose.position.y = y
        q = R.from_euler("z", th).as_quat()
        ps.pose.orientation.x = q[0]
        ps.pose.orientation.y = q[1]
        ps.pose.orientation.z = q[2]
        ps.pose.orientation.w = q[3]
        path.poses.append(ps)

    traj_pub.publish(path)


def pubGoal(goal_robot):
    x, y, th = goal_robot
    ps = PoseStamped()
    ps.header.stamp = rospy.Time.now()
    ps.header.frame_id = "base_link"
    ps.pose.position.x = x
    ps.pose.position.y = y
    q = R.from_euler("z", th).as_quat()
    ps.pose.orientation.x = q[0]
    ps.pose.orientation.y = q[1]
    ps.pose.orientation.z = q[2]
    ps.pose.orientation.w = q[3]
    goal_pub.publish(ps)


def rotate_towards_goal(robot_pose, goal_pose):
    """
    Rotate robot in place until orientation matches goal.
    """
    err = wrap_angle(goal_pose[2] - robot_pose[2])
    K = 1.2
    w = np.clip(K*err, -1.8, 1.8)

    msg = Twist()
    msg.linear.x = 0.0
    msg.angular.z = w
    cmd_pub.publish(msg)

    return abs(err) <= GOAL_ANGLE_THRESHOLD


# -------------------------------------------------------------------
# Parameters
# -------------------------------------------------------------------

ts = 0.5
horizon = 5
robotModelPT2 = PT2Block(ts=ts, T=0.05, D=0.8)

# -------------------------------------------------------------------
# Wait for global path
# -------------------------------------------------------------------

print("Waiting for global path...")
while not path_received and not rospy.is_shutdown():
    rospy.sleep(0.1)

if not path_received:
    rospy.signal_shutdown("No global path received")
    exit(1)

print("Global path received. Starting local planner...")

# -------------------------------------------------------------------
# Main loop
# -------------------------------------------------------------------

rate = rospy.Rate(1.0/ts)
last_control = np.array([0.0, 0.0])

while not rospy.is_shutdown():

    if not global_path or current_goal_ID >= len(global_path):
        pubCMD([0.0, 0.0])
        rate.sleep()
        continue

    robotpose = localiseRobot()
    if robotpose is None:
        rate.sleep()
        continue

    current_goal = np.array(global_path[current_goal_ID])
    dist = np.linalg.norm(robotpose[:2] - current_goal[:2])
    ang  = abs(wrap_angle(robotpose[2] - current_goal[2]))

    # Rotate in place if position reached but angle not
    if dist <= GOAL_DISTANCE_THRESHOLD and ang > GOAL_ANGLE_THRESHOLD:
        if rotate_towards_goal(robotpose, current_goal):
            current_goal_ID += 1
        rate.sleep()
        continue

    # Fully reached
    if dist <= GOAL_DISTANCE_THRESHOLD and ang <= GOAL_ANGLE_THRESHOLD:
        if current_goal_ID < len(global_path)-1:
            current_goal_ID += 1
        else:
            final_goal_reached = True

    if final_goal_reached:
        pubCMD([0.0, 0.0])
        rate.sleep()
        continue

    goal_robot = transform_goal_to_robot_frame(robotpose, global_path[current_goal_ID])

    controls = generateControls(last_control)
    costs, trajectories = evaluateControls(controls, robotModelPT2, horizon, goal_robot)

    best_idx = np.argmin(costs)
    best_control = controls[best_idx]
    best_traj = trajectories[best_idx]

    publish_all_trajectories(trajectories, costs)
    publish_best_trajectory(best_traj)
    pubCMD(best_control)
    pubTrajectory(best_traj)
    pubGoal(goal_robot)

    last_control = best_control
    rate.sleep()
