#include "my_rosplan_interface/moving_action.h"
#include <unistd.h>
#include <visualization_msgs/MarkerArray.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <erl2/PlanningAction.h>

namespace KCL_rosplan {
	float x0;
	float y0;
	float x1;
	float y1;
	float x2;
	float y2;
	float x3;
	float y3;
	
	MovingActionInterface::MovingActionInterface(ros::NodeHandle &nh) {
	// here the initialization
	}
	
	bool MovingActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
	// here the implementation of the action
	std::cout << "MOVING ACTION: Going from " << msg->parameters[0].value << " to " << msg->parameters[1].value << std::endl;


	actionlib::SimpleActionClient<erl2::PlanningAction> ac("/reaching_goal", true);
	erl2::PlanningGoal goal;
	ac.waitForServer();
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
	ac.sendGoal(goal);
	ac.waitForResult();



	ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
	return true;
	}
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "movingaction", ros::init_options::AnonymousName);
	ros::NodeHandle nh("~");
	KCL_rosplan::MovingActionInterface my_aci(nh);
	my_aci.runActionInterface();
	
	return 0;
}

