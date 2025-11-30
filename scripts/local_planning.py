## Imports required components and manually creates a complex global path for debugging
import numpy as np
import numpy.typing as npt
import rospy
import tf2_ros
from scipy.spatial.transform import Rotation as R
import copy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header
from geometry_msgs.msg import Pose


global_path = [
    [2.5 ,1, 0],
    [3, 1.5, 0.5*np.pi],
    [2.5, 2, np.pi],
    [1.5, 2, np.pi],
    [1, 1.5, 1.5*np.pi],
    [1, 0.5, 1.5*np.pi],
    [0.5, 0, np.pi],
    [0, 0, np.pi],
]

current_goal_ID = 1

# Current goal as a numpy array (map frame)
goalpose = np.array(global_path[current_goal_ID])
print(f"Current goalpose for debugging: {goalpose}")


## Initialises a ROS node and required transform buffer objects for robot localisation

try:
    rospy.init_node("local_planner") # <- If not already initialised, create your node here
except: pass

tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

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

robotpose = localiseRobot()
print(robotpose)

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
    """
    Converts a 3x3 transform matrix back into a pose (x, y, theta)
    using atan2 for the rotation extraction.
    """
    x = T[0, 2]
    y = T[1, 2]

    # Extract theta = atan2(sin, cos)
    theta = np.atan2(T[1, 0], T[0, 0])

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
    v_min, v_max = -0.1, 0.2       # m/s   (reverse to slow forward)
    w_min, w_max = -1.4, 1.4       # rad/s

    # How far we are allowed to deviate from previous control
    v_delta_max = 0.03             # m/s per timestep
    w_delta_max = 0.4              # rad/s per timestep

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
    num_v_samples = 7
    num_w_samples = 7

    v_values = np.linspace(v_low, v_high, num_v_samples)
    w_values = np.linspace(w_low, w_high, num_w_samples)

    # -----------------------------
    # Form all combinations
    # -----------------------------
    controls = np.array([[v, w] for v in v_values for w in w_values])

    return controls


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


def costFn(pose: npt.ArrayLike,
           goalpose: npt.ArrayLike,
           control: npt.ArrayLike) -> float:
    """
    Computes the cost of reaching `pose` when trying to reach `goalpose`
    using control input `control`.

    pose:      np.array([x, y, theta])
    goalpose:  np.array([xg, yg, thetag])
    control:   np.array([vt, wt])

    Returns:
        cost: float (lower is better)
    """

    pose      = np.asarray(pose)
    goalpose  = np.asarray(goalpose)
    control   = np.asarray(control)

    # ------------------------------------------------------------
    # 1. Compute state error       e = x_robot - x_goal
    # ------------------------------------------------------------
    e = pose - goalpose

    # Wrap angle difference
    e[2] = wrap_angle(e[2])

    # Take absolute values (element-wise)
    e = np.abs(e)

    # ------------------------------------------------------------
    # 2. State weighting matrix Q
    # ------------------------------------------------------------
    Q = np.array([
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 0.5]
    ])

    # ------------------------------------------------------------
    # 3. Control weighting matrix R
    # ------------------------------------------------------------
    R = np.array([
        [0.1, 0.0],
        [0.0, 0.1]
    ])

    # Absolute values of control
    u = np.abs(control)

    # ------------------------------------------------------------
    # 4. Compute cost:
    #
    #     c = e^T Q e  +  u^T R u
    # ------------------------------------------------------------
    
    cost = e.T @ (Q @ e) + u.T @ (R @ u)

    return float(cost)

def evaluateControls(controls, robotModelPT2, horizon, goalpose, ts, start_pose=None):
    """
    Evaluate a list of candidate controls by forward-simulating the PT2 robot model.

    controls: iterable of [v, w]
    robotModelPT2: PT2Block instance (will be deep-copied per trajectory)
    horizon: number of simulation steps
    goalpose: goal in the same frame as the simulated poses (e.g. robot frame)
    ts: timestep used by forwardKinematics
    start_pose: optional starting pose for simulation (default [0,0,0])

    Returns: (costs, trajectories)
    """
    if start_pose is None:
        start_pose = [0.0, 0.0, 0.0]

    controls_arr = np.asarray(controls)
    costs = np.zeros(controls_arr.shape[0], dtype=float)
    trajectories = [ [] for _ in range(controls_arr.shape[0]) ]

    # Apply range of control signals and compute outcomes
    for ctrl_idx, control in enumerate(controls_arr):
        # Copy currently predicted robot state and PT2 block
        forwardSimPT2 = copy.deepcopy(robotModelPT2)
        forwardpose = np.array(start_pose, dtype=float)

        # Simulate until horizon
        for step in range(horizon):
            control_sim = np.array(control, dtype=float)
            v_t, w_t = control_sim
            # PT2 dynamics on translational velocity only (angular is passed through)
            v_t_dynamic = forwardSimPT2.update(float(v_t))
            control_dym = np.array([v_t_dynamic, w_t], dtype=float)
            forwardpose = forwardKinematics(control_dym, forwardpose, ts)
            costs[ctrl_idx] += costFn(forwardpose, goalpose, control_sim)
            # Track trajectory for visualisation
            trajectories[ctrl_idx].append(forwardpose.copy())

    return costs, trajectories

ts = 0.5 # Sampling time [sec] -> 2Hz
horizon = 10 # Number of time steps to simulate. 10*0.5 sec = 5 seconds lookahead into the future
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
    path_msg.header.frame_id = "map"   # the trajectory is predicted in world/map frame

    for pose in trajectory:
        x, y, theta = pose

        ps = PoseStamped()
        ps.header = path_msg.header
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = 0.0

        # Convert yaw to quaternion
        q = tf.transformations.quaternion_from_euler(0, 0, theta)
        ps.pose.orientation.x = q[0]
        ps.pose.orientation.y = q[1]
        ps.pose.orientation.z = q[2]
        ps.pose.orientation.w = q[3]

        path_msg.poses.append(ps)

    traj_pub.publish(path_msg)


goalpose_robot = transform_goal_to_robot_frame(robotpose, goalpose)


def pubGoal(goalpose_robot):
    x, y, theta = goalpose_robot

    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "base_link"

    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = 0.0

    q = tf.transformations.quaternion_from_euler(0, 0, theta)
    msg.pose.orientation.x = q[0]
    msg.pose.orientation.y = q[1]
    msg.pose.orientation.z = q[2]
    msg.pose.orientation.w = q[3]

    goal_pub.publish(msg)


rate = rospy.Rate(1.0/ts)  # match sampling time

while not rospy.is_shutdown():

    # 1. Localize robot
    robotpose = localiseRobot()

    # 2. Transform goal into robot frame
    goalpose_robot = transform_goal_to_robot_frame(robotpose, global_path[current_goal_ID])

    # 3. Generate controls and evaluate them (in robot frame)
    controls = generateControls(last_control)
    costs, trajectories = evaluateControls(controls, robotModelPT2, horizon, goalpose_robot, ts)

    # 4. Select best
    best_idx = np.argmin(costs)
    best_control = controls[best_idx]
    best_trajectory = trajectories[best_idx]

    # 5. Publish cmd_vel
    pubCMD(best_control)

    # 6. Publish predicted path
    pubTrajectory(best_trajectory)

    # 7. Publish local goal
    pubGoal(goalpose_robot)

    # 8. Remember last control
    last_control = best_control

    rate.sleep()
