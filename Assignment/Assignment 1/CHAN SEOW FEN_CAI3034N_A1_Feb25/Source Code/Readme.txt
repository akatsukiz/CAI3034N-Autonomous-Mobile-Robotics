1) Navigate to Catkin workspace's source directory:
	cd ~/catkin_ws/src

2) Create a new ROS package named turtle_chase with its dependencies:
	catkin_create_pkg turtle_chase roscpp rospy std_msgs geometry_msgs turtlesim

3) Place or create the <prey_node.py> script inside the "turtle_chase/src/" directory.

4) Place or create the <predator_node.py> script inside the "turtle_chase/src/" directory.

5) Inside the newly created turtle_chase package directory, create a directory named launch:
	cd ~/catkin_ws/src/turtle_chase 
	mkdir launch

6) Place or create the <turtle_chase.launch> inside the "turtle_chase/launch/" directory. 

7) In a new terminal, start the ROS Master:
	roscore

8) Split the terminal horizontally, launch the <turtle_chase.launch> file.
	roslaunch turtle_chase turtle_chase.launch