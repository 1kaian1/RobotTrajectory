# MORO Semester Projekt
*By Kai and Magnus*

## Repository Structure
```bash
├───flowcharts/    # Flowchart diagrams
├───launch/        # ROS launch file
├───map/           # ROS map file
├───rviz/          # RViz configuration files
├───screenshots/   # Output of different map visualizations
├───scripts/       # global_planning.py and local_planning.py
└───worlds/        # Simulation world
```
## Requirements
This project uses ROS. Make sure you have a compatible ROS installation before running.
Minimum requirements:
- ROS Noetic
- ```catkin_make``` or appropriate ROS build tool
- RViz for visualization
- Necessary ROS dependencies listed in ```package.xml```

## Installation
1. Clone the repo
```bash
git clone https://github.com/1kaian1/RobotTrajectory.git
cd RobotTrajectory
``` 
2. Build the workspace
```bash
# If using a catkin workspace
mkdir -p ~/catkin_ws/src
cp -r . ~/catkin_ws/src/soarMazeEscape
cd ~/catkin_ws
catkin_make
source devel/setup.bash
chmod +x src/soarMazeEscape/scripts/local_planning.py
chmod +x src/soarMazeEscape/scripts/global_planner.py
```
3. Install dependencies
```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```
### Usage
**Launch simulation and visualization**
```bash
export TURTLEBOT3_MODEL=burger
roslaunch soarMazeEscape launchSimulation.launch
```
When running and RViz is started, publish new 2D Nav Goal in RViz. The global_planning script will then calculate the shortes path to that goal and publish it, so the local_planner script can simulate, create and choose a trajectory which is then executed. 
## Final presentation
[Robot Trajectory.pdf](https://github.com/user-attachments/files/24931164/Robot.Trajectory.pdf)

## Demo video
https://github.com/user-attachments/assets/60fcae83-0873-4ca8-b0a1-b10e308533a5