#include "my_rosplan_interface/goinghome_action.h"
#include <unistd.h>
#include <visualization_msgs/MarkerArray.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <erl2/PlanningAction.h>

namespace KCL_rosplan {
	
	GoingHomeActionInterface::GoingHomeActionInterface(ros::NodeHandle &nh) {
	// here the initialization
	}
	
	bool GoingHomeActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
	// here the implementation of the action
	std::cout << "GOING HOME: Going from " << msg->parameters[0].value << " to " << msg->parameters[1].value << std::endl;


	actionlib::SimpleActionClient<erl2::PlanningAction> ac("/reaching_goal", true);
	
	erl2::PlanningGoal goal;
	ac.waitForServer();
	
		goal.target_pose.pose.position.x = 0.0;
		goal.target_pose.pose.position.y = 0.0;
		goal.target_pose.pose.orientation.w = -3.14/2;
	
	ac.sendGoal(goal);
	ac.waitForResult();



	ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
	return true;
	}
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "goinghomeaction", ros::init_options::AnonymousName);
	ros::NodeHandle nh("~");
	KCL_rosplan::GoingHomeActionInterface my_aci(nh);
	my_aci.runActionInterface();
	
	return 0;
}

