roslaunch soarMazeEscape launchSimulation.launch

sudo apt-get install dos2unix

dos2unix ~/catkin_ws/src/fhtw/soarMazeEscape/scripts/global_planner.py

rosrun soarMazeEscape global_planner.py

-----

fhtw_user@6a77474c71b5:~/catkin_ws/src/fhtw/soarMazeEscape/scripts$ tr -d '\r' < global_planner.py > global_planner_fixed.py
fhtw_user@6a77474c71b5:~/catkin_ws/src/fhtw/soarMazeEscape/scripts$ mv global_planner_fixed.py global_planner.py
