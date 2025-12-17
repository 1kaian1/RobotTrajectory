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


global_path = [
    [2 ,1, 0],
    [2.5, 1, 0],
    [3, 1.5, 0.5*np.pi],
    [2.5, 2, np.pi],
    [2, 2, np.pi],
    [1.5, 2, np.pi],
    [1, 1.5, 1.5*np.pi],
    [1, 0.5, 1.5*np.pi],
    [0.5, 0, np.pi],
    [0, 0, np.pi],
]


current_goal_ID = 0
# Goal reaching thresholds
GOAL_DISTANCE_THRESHOLD = 0.25  # meters
GOAL_ANGLE_THRESHOLD = 0.3    # radians (~11 degrees)

# Current goal as a numpy array (map frame)
goalpose = np.array(global_path[current_goal_ID])
print(f"Current goalpose for debugging: {goalpose}")


## Initialises a ROS node and required transform buffer objects for robot localisation (from cookbook)
try:
    rospy.init_node("local_planner") # <- If not already initialised, create your node here
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
    """
    Converts a pose (x, y, theta) into a 3x3 homogeneous transform.
    pose: array-like [x, y, theta]
    """
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

    # theta = atan2(sin, cos) using Python's math, not NumPy
    theta = math.atan2(T[1, 0], T[0, 0])

    return np.array([x, y, theta])

def transform_goal_to_robot_frame(robot_pose, goal_pose):
    """
    Computes the goal pose expressed in the robot's coordinate frame.
    
    robot_pose: (x, y, theta) in map frame
    goal_pose:  (x, y, theta) in map frame
    
    Returns:
        goal_in_robot_frame = (x, y, theta)
    """
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
    """
    Generates a list of valid (vt, wt) control pairs based on the last applied control.
    lastControl: [v_last, w_last]

    Returns:
        controls: array of shape (N, 2)
    """

    v_last, w_last = lastControl

    # -----------------------------
    # Robot control constraints
    # -----------------------------
    v_min, v_max = -0.2, 0.2       # m/s   (reverse to slow forward)
    w_min, w_max = -0.8, 0.8       # rad/s

    # How far we are allowed to deviate from previous control
    v_delta_max = 0.1             # m/s per timestep
    w_delta_max = 0.8            # rad/s per timestep

    # -----------------------------
    # Define allowed local ranges
    # -----------------------------
    v_low  = max(v_min, v_last - v_delta_max)
    v_high = min(v_max, v_last + v_delta_max)

    w_low  = max(w_min, w_last - w_delta_max)
    w_high = min(w_max, w_last + w_delta_max)

    # -----------------------------
    # Sampling resolution
    # fewer samples = faster simulation, less precision
    # -----------------------------
    num_v_samples = 10
    num_w_samples = 25

    v_values = np.linspace(v_low, v_high, num_v_samples)
    w_values = np.linspace(w_low, w_high, num_w_samples)

    # -----------------------------
    # Form all combinations
    # -----------------------------
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
    """Wrap angle into [-pi, pi]."""
    return (theta + np.pi) % (2*np.pi) - np.pi


def check_goal_reached(robot_pose: npt.ArrayLike, goal_pose: npt.ArrayLike) -> bool:
    """
    Check if the robot has reached the goal within acceptable thresholds.
    
    robot_pose: (x, y, theta) in map frame
    goal_pose:  (x, y, theta) in map frame
    
    Returns:
        True if goal is reached, False otherwise
    """
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

    pose      = np.asarray(pose)
    goalpose  = np.asarray(goalpose)
    control   = np.asarray(control)

    # State error
    e = pose - goalpose
    e[2] = wrap_angle(e[2])

    # Distance to goal (used to scale orientation weighting)
    dist = np.linalg.norm(e[:2])

    # Theta weight: high when close, small when far (prevents aggressive early turns)
    theta_base = 0.5
    theta_floor = 0.05
    theta_weight = theta_floor + theta_base * np.exp(-3.0 * dist)

    # State weighting matrix Q
    Q = np.array([
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, float(theta_weight)]
    ])

     # Penalize angular velocity more to avoid excessive turning
    R = np.array([
        [0.1, 0.0],
        [0.0, 0.5]   # increased penalty on angular velocity
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

ts = 1/2 # Sampling time [sec] -> 2Hz
horizon = 5 # Number of time steps to simulate. 10*0.5 sec = 5 seconds lookahead into the future
robotModelPT2 = PT2Block(ts=ts, T=0.05, D=0.8)

cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
traj_pub = rospy.Publisher("/local_plan", Path, queue_size=10)
goal_pub = rospy.Publisher("/local_goal", PoseStamped, queue_size=10)

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
    # NOTE: the simulated trajectories are relative to the robot start_pose (0,0,0),
    # so publish them in the robot frame
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
goalpose_robot = transform_goal_to_robot_frame(robotpose, goalpose)


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



## Visualise transformed goal and robot poses
# Create single figure
plt.rcParams['figure.figsize'] = [7, 7]
fig, ax = plt.subplots()
colourScheme = {
    "darkblue": "#0D3B66",
    "lightblue": "#FAF0CA",
    "orange": "#F4D35E",
    "red": "#EE964B",
    "pink": "#F95738"
}
costs, trajectories = evaluateControls(generateControls([0.0, 0.0]), robotModelPT2, horizon, goalpose_robot)
# Plot robot poses as arrows to indicate orientation
poses = [[0,0,0], goalpose.tolist()]
labels = ["Robot Pose", "Goal Pose"]
colours = [colourScheme["lightblue"], colourScheme["darkblue"]]

for s, l, c in zip(poses, labels, colours):
    # Calculate arrow's dx and dy from s = [x, y, theta]
    p2 = np.array([np.cos(s[2]), np.sin(s[2])])
    # Scale arrow length and draw
    p2 /= np.linalg.norm(p2)*4
    ax.arrow(s[0], s[1], p2[0], p2[1], width=0.02, label=l, color=c)

# Plot walls (from Step 1, if the variable exists)
if 'wallPositions' in locals():
    try: ax.scatter(wallPositions[:,1], wallPositions[:,0], c=colourScheme["darkblue"], alpha=1.0, s=6**2, label="Walls")
    except: pass

# Plot trajectories and shade with costs
costs_scaled = (costs - costs.min())/(costs.max() - costs.min()) # Scales cost from 0-1 and emphasises low costs
costs_inverted = 1-costs_scaled
costs_emphasis = np.exp(3*costs_inverted-3)

for traj, cost in zip(trajectories, costs_emphasis):
    traj = np.array(traj)
    ax.plot(traj[:,0], traj[:,1], c=colourScheme["darkblue"], alpha=(cost))

# Set axes labels and figure title
ax.set_xlabel("X-Coordinate (Robot Coordinate System) [m]")
ax.set_ylabel("Y-Coordinate (Robot Coordinate System [m]")
ax.set_title("Evaluated Trajectories Shaded with Cost")

# Set grid to only plot each metre
ax.set_xticks = [-1, 0, 1]
ax.set_yticks = [-1, 0, 1]
ax.set_xlim(-0, 1.5)
ax.set_ylim(-0.5, 1)

# Move grid behind points
ax.set_axisbelow(True)
ax.grid()

# Add labels
ax.legend()

# Show plot
plt.show()


def rotate_towards_goal(robot_pose, goal_pose):
    """
    Stops translation and rotates robot until orientation error is small.
    """
    angle_error = wrap_angle(goal_pose[2] - robot_pose[2])

    # Proportional turn controller
    K_turn = 1.2 # Increase if turning is too slow
    w = np.clip(K_turn * angle_error, -1.8, 1.8)

    msg = Twist()
    msg.linear.x = 0.0
    msg.angular.z = w
    cmd_pub.publish(msg)

    return abs(angle_error) <= GOAL_ANGLE_THRESHOLD


rate = rospy.Rate(1.0/ts)  # match sampling time
last_control = np.array([0.0, 0.0], dtype=float)
while not rospy.is_shutdown():

    # 1. Localize robot
    robotpose = localiseRobot()

    # 2. Check if current goal is reached
    # For intermediate waypoints, only require position to be within threshold.
    # For the final waypoint, require both position and orientation to be within thresholds.
    current_goal = np.array(global_path[current_goal_ID])
    distance_error = np.linalg.norm(robotpose[:2] - current_goal[:2])
    angle_error = abs(wrap_angle(robotpose[2] - current_goal[2]))

    position_reached = distance_error <= GOAL_DISTANCE_THRESHOLD
    orientation_reached = angle_error <= GOAL_ANGLE_THRESHOLD

    # Debug print to help understand undershoot/goal behavior
    print(f"Goal {current_goal_ID} check -> dist: {distance_error:.3f} m, ang: {angle_error:.3f} rad")

    current_goal = np.array(global_path[current_goal_ID])
    distance_error = np.linalg.norm(robotpose[:2] - current_goal[:2])
    angle_error = abs(wrap_angle(robotpose[2] - current_goal[2]))

    # If the position is reached but not the orientation, rotate in place
    if distance_error <= GOAL_DISTANCE_THRESHOLD and angle_error > GOAL_ANGLE_THRESHOLD:
        print(f"Goal {current_goal_ID}: position OK, rotating to match angle...")
        reached_angle = rotate_towards_goal(robotpose, current_goal)

        # Only move on if angle is correct
        if reached_angle:
            print("Orientation aligned. Moving to next waypoint.")
            current_goal_ID += 1

        rate.sleep()
        continue

    # if position and angle are both within thresholds, move to next waypoint
    if distance_error <= GOAL_DISTANCE_THRESHOLD and angle_error <= GOAL_ANGLE_THRESHOLD:
        print(f"Waypoint {current_goal_ID} fully reached!")

        if current_goal_ID < len(global_path) - 1:
            current_goal_ID += 1
            print(f"Moving to next goal ID: {current_goal_ID}")
        else:
            print("Final goal reached! Stopping.")
            pubCMD(np.array([0.0, 0.0]))
            break


    # 3. Transform goal into robot frame
    goalpose_robot = transform_goal_to_robot_frame(robotpose, global_path[current_goal_ID])

    # 4. Generate controls and evaluate them (in robot frame)
    controls = generateControls(last_control)
    costs, trajectories = evaluateControls(controls, robotModelPT2, horizon, goalpose_robot)

    # 5. Select best
    best_idx = np.argmin(costs)
    best_control = controls[best_idx]
    best_trajectory = trajectories[best_idx]

    # 6. Publish cmd_vel
    pubCMD(best_control)

    # 7. Publish predicted path
    pubTrajectory(best_trajectory)

    # 8. Publish local goal
    pubGoal(goalpose_robot)

    # 9. Remember last control
    last_control = best_control

    rate.sleep()