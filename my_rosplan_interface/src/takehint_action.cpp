#include <ros/ros.h>

#include "my_rosplan_interface/takehint_action.h"
#include <unistd.h>
#include <visualization_msgs/MarkerArray.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <erl2/PlanningAction.h>
#include <erl2/ErlOracle.h>
#include <string>
#include <vector>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <erl2/hints.h>

ros::Publisher pub;
ros::Publisher hints_pub;

namespace KCL_rosplan {
	float z0;
	float z1;
	float z2;
	float z3;
	
	bool present = false;

	int ID;
	std::string ID_str;
	std::string key;
	std::string class_1;


	std::vector<std::string> hint_0;
	std::vector<std::string> hint_1;
	std::vector<std::string> hint_2;
	std::vector<std::string> hint_3;
	std::vector<std::string> hint_4;
	std::vector<std::string> hint_5;
	
	bool known_position_wp0 = false;
	bool known_position_wp1 = false;
	bool known_position_wp2 = false;
	bool known_position_wp3 = false;
	
	TakeHintInterface::TakeHintInterface(ros::NodeHandle &nh) {
	// here the initialization
	}
	
	bool TakeHintInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
		// here the implementation of the action
		std::cout << "TAKING HINT: from " << msg->parameters[0].value << std::endl;
			
		moveit::planning_interface::MoveGroupInterface group("arm");
			group.setEndEffectorLink("cluedo_link");
			group.setPoseReferenceFrame("base_link");
			group.setPlannerId("RRTstar");
			group.setNumPlanningAttempts(10);
			group.setPlanningTime(10.0);
			group.allowReplanning(true);
			group.setGoalJointTolerance(0.0001);
			group.setGoalPositionTolerance(0.0001);
			group.setGoalOrientationTolerance(0.001);
			
			if (msg->parameters[0].value == "wp0" && known_position_wp0 == false){
				group.setNamedTarget("1_25_maybe");
				group.move();
				
				group.setNamedTarget("0_75_maybe");
				group.move();
				
				group.setNamedTarget("zero");
				group.move();

				known_position_wp0 = true;
			}
			
			else if (msg->parameters[0].value == "wp0" && known_position_wp0 == true){
				if (z0 == 1.25){
					group.setNamedTarget("1_25_maybe");
				}
				else if (z0 == 0.75){
					group.setNamedTarget("0_75_maybe");
				}
				group.move();
			}
			
			else if (msg->parameters[0].value == "wp1" && known_position_wp1 == false){
				group.setNamedTarget("1_25_maybe");
				group.move();
				
				group.setNamedTarget("0_75_maybe");
				group.move();
				
				group.setNamedTarget("zero");
				group.move();

				known_position_wp1 = true;
			}
			
			else if (msg->parameters[0].value == "wp1" && known_position_wp1 == true){
				if (z1 == 1.25){
					group.setNamedTarget("1_25_maybe");
				}
				else if (z1 == 0.75){
					group.setNamedTarget("0_75_maybe");
				}
				group.move();
			}
			
			else if (msg->parameters[0].value == "wp2" && known_position_wp2 == false){
				group.setNamedTarget("1_25_maybe");
				group.move();
				
				group.setNamedTarget("0_75_maybe");
				group.move();
				
				group.setNamedTarget("zero");
				group.move();

				known_position_wp2 = true;
			}
			
			else if (msg->parameters[0].value == "wp2" && known_position_wp2 == true){
				if (z2 == 1.25){
					group.setNamedTarget("1_25_maybe");
				}
				else if (z2 == 0.75){
					group.setNamedTarget("0_75_maybe");
				}
				group.move();
			}
			
			else if (msg->parameters[0].value == "wp3" && known_position_wp3 == false){
				group.setNamedTarget("1_25_maybe");
				group.move();
				
				group.setNamedTarget("0_75_maybe");
				group.move();
				
				group.setNamedTarget("zero");
				group.move();


				known_position_wp3 = true;
			}
			
			else if (msg->parameters[0].value == "wp3" && known_position_wp3 == true){
				if (z3 == 1.25){
					group.setNamedTarget("1_25_maybe");
				}
				else if (z3 == 0.75){
					group.setNamedTarget("0_75_maybe");
				}
				group.move();
			}
			
			
			
			
			
		/*	
		if(msg->parameters[0].value == "wp0"){
			if (z0 == 1.25){
				group.setNamedTarget("1_25_maybe");
				}
			else if (z0 == 0.75){
				group.setNamedTarget("0_75_maybe");
				}
			else{
				printf("Z0 NOT WORKING\n");
				group.setNamedTarget("zero");
				}
			group.move(); 
		}
		
		else if (msg->parameters[0].value == "wp1"){
			if (z1 == 1.25){
				group.setNamedTarget("1_25_maybe");
				}
			else if (z1 == 0.75){
				group.setNamedTarget("0_75_maybe");
				}
			else{
				printf("Z1 NOT WORKING\n");
				group.setNamedTarget("zero");
				}
			group.move(); 
		}
		else if (msg->parameters[0].value == "wp2"){
			if (z2 == 1.25){
				group.setNamedTarget("1_25_maybe");
				}
			else if (z2 == 0.75){
				group.setNamedTarget("0_75_maybe");
				}
			else{
				printf("Z2 NOT WORKING\n");
				group.setNamedTarget("zero");
				}
			group.move(); 
		}
		else if (msg->parameters[0].value == "wp3"){		
			if (z3 == 1.25){
				group.setNamedTarget("1_25_maybe");
				}
			else if (z3 == 0.75){
				group.setNamedTarget("0_75_maybe");
				}
			else{
				printf("Z3 NOT WORKING\n");
				group.setNamedTarget("zero");
				}
			group.move();
		}
		*/
	 	else {
	 		std::cout<< "SOMETHING WENT WRONG" << std::endl;
	 	}
	 	
		ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
		return true;
	}
	
	void marker_position_callback(const visualization_msgs::MarkerArray::ConstPtr& msg) {
		z0 = msg->markers[0].pose.position.z;
		z1 = msg->markers[1].pose.position.z;
		z2 = msg->markers[2].pose.position.z;
		z3 = msg->markers[3].pose.position.z;
	}
	
	void oracle_hint_callback(const erl2::ErlOracle::ConstPtr& msg) {
	
		if (hint_0.size() == 4){
			std::cout << "Clearing hint_0" << std::endl;
			hint_0.clear();
		}
		if (hint_1.size() == 4){
			std::cout << "Clearing hint_1" << std::endl;
			hint_1.clear();
		}
		if (hint_2.size() == 4){
			std::cout << "Clearing hint_2" << std::endl;
			hint_2.clear();
		}
		if (hint_3.size() == 4){
			std::cout << "Clearing hint_3" << std::endl;
			hint_3.clear();
		}
		if (hint_4.size() == 4){
			std::cout << "Clearing hint_4" << std::endl;
			hint_4.clear();
		}
		if (hint_5.size() == 4){
			std::cout << "Clearing hint_5" << std::endl;
			hint_5.clear();
		}
		
		ID = msg-> ID;
		key = msg-> key;
		class_1 = msg-> value;
		
		std::cout<< "HINT: "<<std::endl;
		std::cout<< "ID: "<< ID <<std::endl;
		std::cout<< "KEY: "<< key <<std::endl;
		std::cout<< "class_1: "<< class_1 <<std::endl;
				
		if (class_1 != "0" && class_1 != "" && class_1 != "-1")
		{
			if(key == "who" || key == "what" || key == "where") {
				
				if (ID == 0) {
					ID_str = "ID0";
					std::string new_hint;
					new_hint.append(key);
					new_hint.append(":");
					new_hint.append(class_1);
					
					for (int i = 0; i < hint_0.size(); i++) {
						if (hint_0[i] == new_hint){
							present = true;
						}
					}
					if (present == false) {
						hint_0.push_back(new_hint);
						std::cout << "ID_str: " << ID_str << std::endl;
					}
					else{
						std::cout << "Hint already present in hypothesis" << std::endl;
						present = false;
					}
				}
				
				else if (ID == 1) {
					ID_str = "ID1";
					std::string new_hint;
					new_hint.append(key);
					new_hint.append(":");
					new_hint.append(class_1);
					for (int i = 0; i < hint_1.size(); i++) {
						if (hint_1[i] == new_hint){
							present = true;
						}
					}
					if (present == false) { 
						hint_1.push_back(new_hint);
						std::cout << "ID_str: " << ID_str << std::endl;
					}
					else{
						std::cout << "Hint already present in hypothesis" << std::endl;
						present = false;
					}
				}
				
				else if (ID == 2) {
					ID_str = "ID2";
					std::string new_hint;
					new_hint.append(key);
					new_hint.append(":");
					new_hint.append(class_1);
					for (int i = 0; i < hint_2.size(); i++) {
						if (hint_2[i] == new_hint){
							present = true;
						}
					}
					if (present == false) { 
						hint_2.push_back(new_hint);
						std::cout << "ID_str: " << ID_str << std::endl;
					}
					else{
						std::cout << "Hint already present in hypothesis" << std::endl;
						present = false;
					}
				}
				
				else if (ID == 3) {
					ID_str = "ID3";
					std::string new_hint;
					new_hint.append(key);
					new_hint.append(":");
					new_hint.append(class_1);
					for (int i = 0; i < hint_3.size(); i++) {
						if (hint_3[i] == new_hint){
							present = true;
						}
					}
					if (present == false) { 
						hint_3.push_back(new_hint);
						std::cout << "ID_str: " << ID_str << std::endl;
					}
					else{
						std::cout << "Hint already present in hypothesis" << std::endl;
						present = false;
					}
				}
				
				else if (ID == 4) {
					ID_str = "ID4";
					std::string new_hint;
					new_hint.append(key);
					new_hint.append(":");
					new_hint.append(class_1);
					for (int i = 0; i < hint_4.size(); i++) {
						if (hint_4[i] == new_hint){
							present = true;
						}
					}
					if (present == false) { 
						hint_4.push_back(new_hint);
						std::cout << "ID_str: " << ID_str << std::endl;
					}
					else{
						std::cout << "Hint already present in hypothesis" << std::endl;
						present = false;
					}
				}
				
				else if (ID == 5) {
					ID_str = "ID5";
					std::string new_hint;
					new_hint.append(key);
					new_hint.append(":");
					new_hint.append(class_1);
					for (int i = 0; i < hint_5.size(); i++) {
						if (hint_5[i] == (new_hint)){
							present = true;
						}
					}
					if (present == false) { 
						hint_5.push_back(new_hint);
						std::cout << "ID_str: " << ID_str << std::endl;
					}
					else{
						std::cout << "Hint already present in hypothesis" << std::endl;
						present = false;
					}
				}
							
				else {
					std::cout<< "ID error" << std::endl;
				}
				
				erl2::hints hint_msg;
		
				hint_msg.hint_0 = hint_0;
				hint_msg.hint_1 = hint_1;
				hint_msg.hint_2 = hint_2;
				hint_msg.hint_3 = hint_3;
				hint_msg.hint_4 = hint_4;
				hint_msg.hint_5 = hint_5;
				
				hints_pub.publish(hint_msg);
			}
			else {
				std::cout<< "Key error" << std::endl;
			}
			
		}
		else{
			std::cout<< "Value error" << std::endl;
		}
	}
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "takehintaction", ros::init_options::AnonymousName);
	ros::NodeHandle nh("~");
	ros::NodeHandle nh1;
	ros::Subscriber sub = nh1.subscribe("/visualization_marker", 10, KCL_rosplan::marker_position_callback);
	ros::Subscriber hint_sub = nh1.subscribe("/oracle_hint", 10, KCL_rosplan::oracle_hint_callback);
	
	hints_pub = nh1.advertise<erl2::hints>("/hint_list", 0);
	
	KCL_rosplan::TakeHintInterface my_aci(nh);
	my_aci.runActionInterface();
	
	//Moveit
	ros::AsyncSpinner spinner(1);
	spinner.start();
	
	return 0;
}

