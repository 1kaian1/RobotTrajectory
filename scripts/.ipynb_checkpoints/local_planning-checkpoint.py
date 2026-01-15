#!/usr/bin/env python3
## Imports required components and manually creates a complex global path for debugging
import numpy as np
import numpy.typing as npt
import math
import rospy
import tf2_ros
from scipy.spatial.transform import Rotation as R
import copy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
import matplotlib.pyplot as plt
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point



global_path = None
current_goal_ID = 0
path_received = False

def global_path_callback(msg):
    global global_path, path_received, current_goal_ID, final_goal_reached

    # Convert nav_msgs/Path to list of [x, y, theta]
    global_path = []
    for pose_stamped in msg.poses:
        x = pose_stamped.pose.position.x
        y = pose_stamped.pose.position.y
        quat = pose_stamped.pose.orientation
        theta = R.from_quat([quat.x, quat.y, quat.z, quat.w]).as_euler('xyz')[2]
        global_path.append([x, y, theta])
    
    path_received = True
    current_goal_ID = 0          # RESET current goal
    final_goal_reached = False    # RESET final goal flag

    print(f"Global path received with {len(global_path)} waypoints")

    print("global_path2 = [")
    for x, y, theta in global_path:
        print(f"    [{x:.3f}, {y:.3f}, {theta}],")
    print("]\n")




path_sub = rospy.Subscriber("/global_path", Path, global_path_callback)

# Goal reaching thresholds
GOAL_DISTANCE_THRESHOLD = 0.2
GOAL_ANGLE_THRESHOLD = 0.05


## Initialises a ROS node and required transform buffer objects for robot localisation (from cookbook)
try:
    rospy.init_node("local_planner")
except: pass

tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

# From cookbook
def localiseRobot():
    """Localises the robot towards the 'map' coordinate frame. Returns pose in format (x,y,theta)"""
    while True:
        try:
            trans = tfBuffer.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(1.0))
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("Robot localisation took longer than 1 sec")
            continue

    theta = R.from_quat([
        trans.transform.rotation.x,
        trans.transform.rotation.y,
        trans.transform.rotation.z,
        trans.transform.rotation.w]).as_euler("xyz")[2]
    
    return np.array([
        trans.transform.translation.x,
        trans.transform.translation.y,
        theta])

def pose2tf_mat(pose):
    # Converts a pose (x, y, theta) into a 3x3 homogeneous transform.
    # pose: array-like [x, y, theta]
    
    x, y, theta = pose

    c = np.cos(theta)
    s = np.sin(theta)

    T = np.array([
        [c, -s, x],
        [s,  c, y],
        [0,  0, 1]
    ])

    return T

def tf_mat2pose(T):
    x = T[0, 2]
    y = T[1, 2]

    theta = math.atan2(T[1, 0], T[0, 0])

    return np.array([x, y, theta])

def transform_goal_to_robot_frame(robot_pose, goal_pose):    
    # Convert both poses to transform matrices
    T_robot = pose2tf_mat(robot_pose)
    T_goal  = pose2tf_mat(goal_pose)

    # Compute inverse of robot transform
    T_robot_inv = np.linalg.inv(T_robot)

    # Transform goal into robot frame
    T_goal_robot = T_robot_inv @ T_goal

    # Convert back to pose form
    return tf_mat2pose(T_goal_robot)

def generateControls(lastControl: npt.ArrayLike) -> np.ndarray:
    v_last, w_last = lastControl

    # Constraints 
    v_min, v_max = -0.2, 0.2 # m/s   (reverse to slow forward)
    w_min, w_max = -0.8, 0.8 # rad/s

    v_delta_max = 0.1 # m/s per timestep
    w_delta_max = 0.8 # rad/s per timestep

    # Define ranges
    v_low  = max(v_min, v_last - v_delta_max)
    v_high = min(v_max, v_last + v_delta_max)

    w_low  = max(w_min, w_last - w_delta_max)
    w_high = min(w_max, w_last + w_delta_max)

    # Sampling resolution
    num_v_samples = 10
    num_w_samples = 25

    v_values = np.linspace(v_low, v_high, num_v_samples)
    w_values = np.linspace(w_low, w_high, num_w_samples)

    # Form all combinations
    controls = np.array([[v, w] for v in v_values for w in w_values])

    return controls

# FROM COOKBOOK
def forwardKinematics(control: npt.ArrayLike, lastPose: npt.ArrayLike, dt: float, dtype=np.float64) -> np.ndarray:
    """Mobile robot forward kinematics (see Thrun Probabilistic Robotics)
    """
    if not isinstance(lastPose, np.ndarray):  # Check input formatting
        lastPose = np.array(lastPose, dtype=dtype)
    assert lastPose.shape == (3,), "Wrong pose format. Pose must be provided as list or array of form [x, y, theta]"
    if not isinstance(control, np.ndarray): control = np.array(control)
    assert control.shape == (2,), "Wrong control format. Control must be provided as list or array of form [vt, wt]"
    vt, wt = control
    # Set omega to smallest possible nonzero value in case it is zero to avoid division by zero
    if wt == 0: wt = np.finfo(dtype).tiny
    vtwt = vt/wt
    _, _, theta = lastPose
    return lastPose + np.array([
        -vtwt*np.sin(theta) + vtwt*np.sin(theta + (wt*dt)),
        vtwt*np.cos(theta) - vtwt*np.cos(theta + (wt*dt)),
        wt*dt
    ], dtype=dtype)

# From cookbook
class PT2Block:
    """Discrete PT2 Block approximated using the Tustin approximation (rough robot dynamics model)
    """
    def __init__(self, T=0, D=0, kp=1, ts=0, bufferLength=3) -> None:
        self.k1, self.k2, self.k3, self.k4, self.k5, self.k6 = 0, 0, 0, 0, 0, 0
        self.e = [0 for i in range(bufferLength)]
        self.y = [0 for i in range(bufferLength)]
        if ts != 0:  self.setConstants(T, D, kp, ts)
    #
    def setConstants(self, T, D, kp, ts) -> None:
        self.k1 = 4*T**2 + 4*D*T*ts + ts**2
        self.k2 = 2*ts**2 - 8*T**2
        self.k3 = 4*T**2 - 4*D*T*ts + ts**2
        self.k4 = kp*ts**2
        self.k5 = 2*kp*ts**2
        self.k6 = kp*ts**2
    #
    def update(self, e) -> float:    
        self.e = [e]+self.e[:len(self.e)-1] # Update buffered input and output signals
        self.y = [0]+self.y[:len(self.y)-1]
        e, y = self.e, self.y # Shorten variable names for better readability
        # Calculate output signal and return output
        y[0] = ( e[0]*self.k4 + e[1]*self.k5 + e[2]*self.k6 - y[1]*self.k2 - y[2]*self.k3 )/self.k1
        return y[0]

def wrap_angle(theta: float) -> float:
    # Wrap angle into [-pi, pi]. 
    return (theta + np.pi) % (2*np.pi) - np.pi


def check_goal_reached(robot_pose: npt.ArrayLike, goal_pose: npt.ArrayLike) -> bool:
    robot_pose = np.asarray(robot_pose)
    goal_pose = np.asarray(goal_pose)
    
    # Compute distance error
    distance_error = np.linalg.norm(robot_pose[:2] - goal_pose[:2])
    
    # Compute angle error
    angle_error = abs(wrap_angle(robot_pose[2] - goal_pose[2]))
    
    # Check both thresholds
    return (distance_error <= GOAL_DISTANCE_THRESHOLD and 
            angle_error <= GOAL_ANGLE_THRESHOLD)


def costFn(pose: npt.ArrayLike,
           goalpose: npt.ArrayLike,
           control: npt.ArrayLike) -> float:

    pose = np.asarray(pose)
    goalpose = np.asarray(goalpose)
    control = np.asarray(control)

    # State error
    e = pose - goalpose
    e[2] = wrap_angle(e[2])

    # Distance to goal (used to scale orientation weighting)
    dist = np.linalg.norm(e[:2])

    # State weighting matrix Q
    Q = np.array([
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 0.1]
    ])

     # Penalize angular velocity more to avoid excessive turning
    R = np.array([
        [0.1, 0.0],
        [0.0, 0.5]
    ])

    u = control

    cost = e.T @ (Q @ e) + u.T @ (R @ u)

    return float(cost)

# From cookbook
def evaluateControls(controls, robotModelPT2, horizon, goalpose_local):
    costs = np.zeros_like(np.array(controls)[:,0], dtype=float)
    trajectories = [ [] for _ in controls ]
    
    # Apply range of control signals and compute outcomes
    for ctrl_idx, control in enumerate(controls):
    
        # Copy currently predicted robot state
        forwardSimPT2 = copy.deepcopy(robotModelPT2)
        forwardpose = [0,0,0]
    
        # Simulate until horizon
        for step in range(horizon):
            control_sim = copy.deepcopy(control)
            v_t, w_t = control
            v_t_dynamic = forwardSimPT2.update(v_t)
            control_dym = [v_t_dynamic, w_t]
            forwardpose = forwardKinematics(control_dym, forwardpose, ts)
            costs[ctrl_idx] += costFn(forwardpose, goalpose_local, control_sim)
            # Track trajectory for visualisation
            trajectories[ctrl_idx].append(forwardpose)

    return costs, trajectories

def publish_all_trajectories(trajectories, costs):
    marker_array = MarkerArray()

    min_cost = min(costs)
    max_cost = max(costs)
    cost_range = max_cost - min_cost if max_cost != min_cost else 1.0

    for i, traj in enumerate(trajectories):
        m = Marker()
        m.header.frame_id = "base_link"   # nebo "map" podle toho, v čem počítáš
        m.header.stamp = rospy.Time.now()
        m.ns = "local_trajectories"
        m.id = i
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.01   # tloušťka čáry

        # Normalizace ceny 0..1
        norm = (costs[i] - min_cost) / cost_range

        # Barva: zelená (dobrá) -> červená (špatná)
        m.color.r = norm
        m.color.g = 1.0 - norm
        m.color.b = 0.0
        m.color.a = 0.6

        for pose in traj:
            p = Point()
            p.x = pose[0]
            p.y = pose[1]
            p.z = 0.02
            m.points.append(p)

        marker_array.markers.append(m)

    traj_viz_pub.publish(marker_array)

def publish_best_trajectory(traj):
    m = Marker()
    m.header.frame_id = "base_link"
    m.header.stamp = rospy.Time.now()
    m.ns = "best_trajectory"
    m.id = 0
    m.type = Marker.LINE_STRIP
    m.action = Marker.ADD
    m.scale.x = 0.03

    m.color.r = 0.0
    m.color.g = 0.0
    m.color.b = 1.0
    m.color.a = 1.0

    for pose in traj:
        p = Point()
        p.x = pose[0]
        p.y = pose[1]
        p.z = 0.03
        m.points.append(p)

    traj_viz_pub.publish(MarkerArray(markers=[m]))


ts = 1/2 # Sampling time [sec] -> 2Hz
horizon = 5 # Number of time steps to simulate. 10*0.5 sec = 5 seconds lookahead into the future
robotModelPT2 = PT2Block(ts=ts, T=0.05, D=0.8)

cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
traj_pub = rospy.Publisher("/local_plan", Path, queue_size=10)
goal_pub = rospy.Publisher("/local_goal", PoseStamped, queue_size=10)
traj_viz_pub = rospy.Publisher(
    "/all_local_trajectories",
    MarkerArray,
    queue_size=1
)


def pubCMD(best_control):
    v, w = best_control

    msg = Twist()
    msg.linear.x  = float(v)
    msg.angular.z = float(w)

    cmd_pub.publish(msg)

def pubTrajectory(trajectory):
    path_msg = Path()
    path_msg.header = Header()
    path_msg.header.stamp = rospy.Time.now()
    path_msg.header.frame_id = "base_link"

    for pose in trajectory:
        x, y, theta = pose

        ps = PoseStamped()
        ps.header = path_msg.header
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = 0.0

        # Use scipy Rotation to create quaternion [x,y,z,w]
        q = R.from_euler('z', float(theta)).as_quat()  # returns [x, y, z, w]
        ps.pose.orientation.x = float(q[0])
        ps.pose.orientation.y = float(q[1])
        ps.pose.orientation.z = float(q[2])
        ps.pose.orientation.w = float(q[3])

        path_msg.poses.append(ps)

    traj_pub.publish(path_msg)

robotpose = localiseRobot()
#goalpose_robot = transform_goal_to_robot_frame(robotpose, goalpose)

def pubGoal(goalpose_robot):
    x, y, theta = goalpose_robot

    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "base_link"

    msg.pose.position.x = float(x)
    msg.pose.position.y = float(y)
    msg.pose.position.z = 0.0

    q = R.from_euler('z', float(theta)).as_quat()  # [x,y,z,w]
    msg.pose.orientation.x = float(q[0])
    msg.pose.orientation.y = float(q[1])
    msg.pose.orientation.z = float(q[2])
    msg.pose.orientation.w = float(q[3])

    goal_pub.publish(msg)


def rotate_towards_goal(robot_pose, goal_pose):
    # Stops translation and rotates robot until orientation error is small.
    angle_error = wrap_angle(goal_pose[2] - robot_pose[2])

    K_turn = 1.2 # Increase if turning is too slow
    w = np.clip(K_turn * angle_error, -1.8, 1.8)

    msg = Twist()
    msg.linear.x = 0.0
    msg.angular.z = w
    cmd_pub.publish(msg)

    return abs(angle_error) <= GOAL_ANGLE_THRESHOLD

print("Waiting for global path...")
while not path_received and not rospy.is_shutdown():
    rospy.sleep(0.1)

if not path_received:
    rospy.logfatal("No global path received, shutting down local planner.")
    rospy.signal_shutdown("No global path received")
    exit(1)

print("Global path received. Starting local planner...")

rate = rospy.Rate(1.0/ts)  # match sampling time
last_control = np.array([0.0, 0.0], dtype=float)
final_goal_reached = False
current_goal_ID = 0

while not rospy.is_shutdown():

    # --- check if global path is empty or current_goal_ID out of bounds ---
    if not global_path or current_goal_ID >= len(global_path):
        # stop the robot but keep simulation alive
        pubCMD(np.array([0.0, 0.0]))
        rospy.sleep(0.1)
        continue  # just wait for a valid path

    # Localize robot
    robotpose = localiseRobot()

    # Current goal
    current_goal = np.array(global_path[current_goal_ID])
    distance_error = np.linalg.norm(robotpose[:2] - current_goal[:2])
    angle_error = abs(wrap_angle(robotpose[2] - current_goal[2]))

    # Rotate in place if position reached but angle not
    if distance_error <= GOAL_DISTANCE_THRESHOLD and angle_error > GOAL_ANGLE_THRESHOLD:
        reached_angle = rotate_towards_goal(robotpose, current_goal)
        if reached_angle:
            current_goal_ID += 1
        rate.sleep()
        continue

    # If position and angle within threshold
    if distance_error <= GOAL_DISTANCE_THRESHOLD and angle_error <= GOAL_ANGLE_THRESHOLD:
        if current_goal_ID < len(global_path) - 1:
            current_goal_ID += 1
        else:
            final_goal_reached = True

    # Stop robot if final goal reached
    if final_goal_reached:
        pubCMD(np.array([0.0, 0.0]))
        rate.sleep()
        continue  # keep node alive

    # Transform goal into robot frame
    goalpose_robot = transform_goal_to_robot_frame(robotpose, global_path[current_goal_ID])

    # Generate controls and evaluate them
    controls = generateControls(last_control)
    costs, trajectories = evaluateControls(controls, robotModelPT2, horizon, goalpose_robot)

    best_idx = np.argmin(costs)
    best_control = controls[best_idx]
    best_trajectory = trajectories[best_idx]

    publish_all_trajectories(trajectories, costs)
    publish_best_trajectory(best_trajectory)
    pubCMD(best_control)
    pubTrajectory(best_trajectory)
    pubGoal(goalpose_robot)
    last_control = best_control

    rate.sleep()
