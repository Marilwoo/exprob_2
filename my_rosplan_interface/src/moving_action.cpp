/**
* \file moving_action.cpp
* \brief Node for moving between waypoints
* \author Maria Luisa Aiachini
*
* \details This node is used for moving the robot bewteen waypoints.
*
* Action Client: <BR>
*	°/reaching_goal
*
* Description:
*
* This node takes the name of the waypoint the robot has to reach from the msg
* and assigns to it the x and y coordinates as well as the orientation. It then uses the Action Client
* /reaching_goal, of type PlanningAction, to make the robot move by sending the coordinates.
* 
*
*/

#include "my_rosplan_interface/moving_action.h"
#include <unistd.h>
#include <visualization_msgs/MarkerArray.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <erl2/PlanningAction.h>

namespace KCL_rosplan {
	
	MovingActionInterface::MovingActionInterface(ros::NodeHandle &nh) {
	// here the initialization
	}
	
/**
* \param msg: containing information about the markers.
*
* \return always true.
*
* This function takes the name of the waypoint, assigns it the x and y coordinates and the orientation to reach.
* It then uses the Action Client /reaching_goal to make the robot move.
*
*/
	bool MovingActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
	// here the implementation of the action
	std::cout << '\n'<< '\n';
	std::cout << "MOVING ACTION: Going from " << msg->parameters[0].value << " to " << msg->parameters[1].value << std::endl;
	std::cout << '\n'<< '\n';

	//Initializing the Action Client
	actionlib::SimpleActionClient<erl2::PlanningAction> ac("/reaching_goal", true);
	erl2::PlanningGoal goal;
	ac.waitForServer();
	
	//Assigning coordinates and orientation depending on the waypoint to reach
	if(msg->parameters[1].value == "wp0"){
		goal.target_pose.pose.position.x = -2.1;
		goal.target_pose.pose.position.y = 0.0;
		goal.target_pose.pose.orientation.w = 3.14;
	}
	else if (msg->parameters[1].value == "wp1"){
		goal.target_pose.pose.position.x = 2.1;
		goal.target_pose.pose.position.y = 0.0;
		goal.target_pose.pose.orientation.w =  0.0;
	}
	else if (msg->parameters[1].value == "wp2"){
		goal.target_pose.pose.position.x = 0.0;
		goal.target_pose.pose.position.y = -2.1;
		goal.target_pose.pose.orientation.w =  -3.14/2;
	}
	else if (msg->parameters[1].value == "wp3"){
		goal.target_pose.pose.position.x = 0.0;
		goal.target_pose.pose.position.y = 2.1;
		goal.target_pose.pose.orientation.w = 3.14/2;
	}
	
	//Sending goal to the Action Client
	ac.sendGoal(goal);
	ac.waitForResult();

	std::cout << '\n'<< '\n';
	ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
	std::cout << '\n'<< '\n';
	return true;
	}
}

/**
* \param argc
* \param argv
*
* \return  always 0
*
* Main function for the movingaction node.
* Here the initialization of the node and of the action for rosplan.
*
*/
int main(int argc, char **argv) {

	//Initializing the node
	ros::init(argc, argv, "movingaction", ros::init_options::AnonymousName);
	ros::NodeHandle nh("~");
	KCL_rosplan::MovingActionInterface my_aci(nh);
	my_aci.runActionInterface();
	
	return 0;
}

