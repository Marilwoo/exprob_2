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
- [my_rosplan_interface](https://github.com/Marilwoo/exprob_2/tree/master/my_rosplan_interface)
- [exprob_2_moveit](https://github.com/Marilwoo/exprob_2/tree/master/exprob_2_moveit)

#### Temporal diagram
#### UML and packages description
#### ROS msgs
#### ROS srv

### Installation and how to run

### Working description, screenshots 

### Documentation

### System features

### System limitations

### Possible improvements

### Contacts
Maria Luisa Aiachini - 4375373
4375373@studenti.unige.it
