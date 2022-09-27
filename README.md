# Experimental Robotics Laboratory - Second assignment
Maria Luisa Aiachini - 4375373

### General introduction
This project simulates a simple version of cluedo. The robot is inside a room enclosed by four walls; the robot needs to go around the room, searching for some hints and then, when it has a complete and consistent hypothesis it needs to check if it is the winning one; if it is the winning one the simulation stops, if not, the robot goes back to find new hints.
The hints the robot has to take are placed in defined positions: (3,0)(-3,0)(0,3)(0,-3), but the height of the hint is randomly chosen, at the start of the simulation, between two positions: 0.75 and 1.25; this means that the robot should be able to move the arm to these two positions to be able to take the hints.

### Software architecture
For this project are needed three packages:
- [erl2](https://github.com/Marilwoo/exprob_2/tree/master/erl2) This node contains:
	- Main launch file for the project
	- Go_to_point script for managing the robot movement
	- Planning script for managing the replanning when the plan does not end with a success
	- Simulation file that manages the position and the sending of the hints
	- Urdf and xacro files for the robot
	- Pddl files for domain and problem
	- [Documentation](https://github.com/Marilwoo/exprob_2/tree/master/erl2/docs/html)
- [my_rosplan_interface](https://github.com/Marilwoo/exprob_2/tree/master/my_rosplan_interface) This node the action files for the ROSplan:
	- Move from home
	- Move between waypoints
	- Move towards home
	- Take hints
	- Check hints
	- [Documentation](https://github.com/Marilwoo/exprob_2/tree/master/my_rosplan_interface/docs/html)
- [exprob_2_moveit](https://github.com/Marilwoo/exprob_2/tree/master/exprob_2_moveit)
	- Configuration files for moveit
	- Launch files for moveit
	
#### UML and packages description
#### Temporal diagram
#### ROS msgs
- ErlOracle.msg
	```
	int32 ID
	string key
	string value
	```
- hints.msg
	```
	string[] hint_0
	string[] hint_1
	string[] hint_2
	string[] hint_3
	string[] hint_4
	string[] hint_5
	```
#### ROS srv
- Check_srv.srv
	```
	string ID_srv
	---
	bool check
	```
- Oracle.srv
	```
	---
	int32 ID
	```

### Installation and how to run
For this project to work are needed:
- The three packages 
	- [erl2](https://github.com/Marilwoo/exprob_2/tree/master/erl2)
	- [my_rosplan_interface](https://github.com/Marilwoo/exprob_2/tree/master/my_rosplan_interface)
	- [exprob_2_moveit](https://github.com/Marilwoo/exprob_2/tree/master/exprob_2_moveit)
These packages need to be cloned in your ros_ws ROS workspace

- ROSplan

To build the code run:
	```
	catkin_make -DCATKIN_WHITELIST_PACKAGES=""
	```
To run the code in a terminal run:
	```
	roslaunch erl2 mylaunch.launch
	```


### Working description, screenshots

### System features

### System limitations

### Possible improvements

### Contacts
Maria Luisa Aiachini - 4375373
4375373@studenti.unige.it
